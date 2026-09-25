"""Camera stream for the hardware backend: H.264 bytes -> RGB numpy frames.

Sources of H.264 (Annex-B byte stream):
  'plutocam'          the Pluto WiFi camera module, through the plutocam
                      library (pip install plutocam); camera at 192.168.0.1
  'tcp://HOST:PORT'   a raw H.264 byte stream over TCP (the simulator's
                      MSP test bridge serves one)

Decoding: an ffmpeg subprocess (h264 in, rgb24 rawvideo out), with
low-delay flags. The frame size is read from ffmpeg's stream info.
Frames get the host's monotonic receive time (the drone's capture time is
not known on this link) and a sequence number.
"""

from __future__ import annotations

import re
import shutil
import socket
import subprocess
import threading
import time
from typing import Callable, Iterator, Optional

import numpy as np

from avatic_drone.types import Frame


def plutocam_source(host: str = '192.168.0.1') -> Iterator[bytes]:
    # imported here, not in the generator, so a missing library fails at startup
    try:
        from plutocam import LWDrone
    except ImportError as error:
        raise ImportError('the Pluto camera needs plutocam: pip install plutocam') from error
    camera = LWDrone(host) if host else LWDrone()
    return (frame.frame_bytes for frame in camera.start_video_stream())


def tcp_source(host: str, port: int, stop: threading.Event) -> Iterator[bytes]:
    sock = socket.create_connection((host, port), timeout=5.0)
    sock.settimeout(0.5)
    try:
        while not stop.is_set():
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                return
            yield data
    finally:
        sock.close()


class VideoStream:
    """Background decoding of an H.264 source; get_frame() = latest frame."""

    def __init__(self, source: str = 'plutocam', host: str = '192.168.0.1',
                 clock: Callable[[], float] = time.monotonic):
        if shutil.which('ffmpeg') is None:
            raise RuntimeError('ffmpeg is needed to decode the camera stream '
                               '(Linux: sudo apt install ffmpeg)')
        self._clock = clock
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[Frame] = None
        self._seq = 0
        self._size = None
        self._size_ready = threading.Event()
        self.error: Optional[str] = None
        self._error_reported = False
        if source == 'plutocam':
            chunks = plutocam_source(host)
        elif source.startswith('tcp://'):
            host, _, port = source[len('tcp://'):].rpartition(':')
            chunks = tcp_source(host, int(port), self._stop)
        else:
            raise ValueError(f"video source must be 'plutocam' or 'tcp://host:port', got {source!r}")
        self._ffmpeg = subprocess.Popen(
            ['ffmpeg', '-hide_banner', '-nostats', '-loglevel', 'info', '-fflags', 'nobuffer',
             '-flags', 'low_delay', '-probesize', '32768', '-analyzeduration', '0',
             '-f', 'h264', '-i', 'pipe:0', '-f', 'rawvideo', '-pix_fmt', 'rgb24', 'pipe:1'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._threads = [threading.Thread(target=f, daemon=True) for f in
                         (lambda: self._feed(chunks), self._read_info, self._read_frames)]
        for thread in self._threads:
            thread.start()

    def _feed(self, chunks: Iterator[bytes]):
        try:
            for chunk in chunks:
                if self._stop.is_set():
                    break
                self._ffmpeg.stdin.write(chunk)
                self._ffmpeg.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            if not self._stop.is_set():
                self.error = f'video source stopped: {error}'
        except Exception as error:  # noqa: BLE001 - camera library errors
            self.error = f'video source failed: {error!r}'
        finally:
            try:
                self._ffmpeg.stdin.close()
            except OSError:
                pass

    def _read_info(self):
        pattern = re.compile(rb'Video: .*?(\d{2,5})x(\d{2,5})')
        for line in iter(self._ffmpeg.stderr.readline, b''):
            match = pattern.search(line)
            if match and not self._size_ready.is_set():
                self._size = (int(match.group(1)), int(match.group(2)))
                self._size_ready.set()

    def _read_frames(self):
        while not self._size_ready.wait(0.2):
            if self._stop.is_set() or self._ffmpeg.poll() is not None:
                return
        width, height = self._size
        frame_bytes = width * height * 3
        stdout = self._ffmpeg.stdout
        while not self._stop.is_set():
            data = stdout.read(frame_bytes)
            if len(data) < frame_bytes:
                if not self._stop.is_set() and self.error is None:
                    self.error = 'the video decoder (ffmpeg) stopped'
                return
            image = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3).copy()  # writable, as in the sim
            with self._lock:
                self._seq += 1
                self._latest = Frame(image=image, time_s=self._clock(), width=width,
                                     height=height, seq=self._seq)

    def get_frame(self) -> Optional[Frame]:
        """The latest frame (None before the first). After a video failure
        the last frame stays; the failure is printed once."""
        if self.error and not self._error_reported:
            self._error_reported = True
            print(f'[hitl] CAMERA: {self.error} - get_frame() returns the last frame',
                  flush=True)
        with self._lock:
            return self._latest

    def close(self):
        self._stop.set()
        try:
            self._ffmpeg.kill()
            self._ffmpeg.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            pass
        for thread in self._threads:
            thread.join(timeout=1.0)
