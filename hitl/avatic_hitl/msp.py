"""MSP v1 client for the Pluto X (MagisV2 firmware).

Frame format (MultiWii Serial Protocol v1):
    request  '$' 'M' '<'  size  cmd  payload[size]  checksum
    reply    '$' 'M' '>'  size  cmd  payload[size]  checksum
    error    '$' 'M' '!'  size  cmd  payload[size]  checksum
    checksum = size XOR cmd XOR every payload byte; multi-byte values are
    little-endian.

Message layouts below are taken from the MagisV2 source
(firmware/magisv2/src/main/io/serial_msp.cpp), not from generic
MultiWii/Cleanflight documentation - several differ:

  MSP_STATUS 101 (11 bytes)
      u16 flightIndicatorFlag   Pluto status bits (FlightStatus_e,
                                config/runtime_config.h): 0 accel/gyro
                                calibration, 1 mag calibration, 2 crash,
                                3 low battery, 4 low battery in flight,
                                5 signal loss, 6 not ok to arm, 7 ok to arm,
                                8 armed
      u16 i2c error count
      u16 sensors               bit 0 acc, 1 baro, 2 mag, 3 gps, 4 sonar
      u32 active mode flags     bit i = activeBoxIds[i] is active; ARM is
                                always activeBoxIds[0] (serial_msp.cpp mspInit)
      u8  profile
  MSP_ATTITUDE 108 (6)   i16 roll decideg (+ right side down),
                         i16 pitch decideg (+ NOSE DOWN), i16 heading deg
  MSP_ALTITUDE 109 (6)   i32 estimated altitude cm, i16 vario cm/s
  MSP_ANALOG 110 (10)    u16 battery mV (vBatComp), u16 current (mAmpRaw),
                         u16 mAh drawn, u16 mAh remaining, u8 state of
                         charge %, u8 battery warning mode
  MSP_BOXIDS 119         u8 permanent box id per active mode (index = bit
                         in the MSP_STATUS mode flags)
  MSP_SET_RAW_RC 200     u16 x N channels (AETR1234, microseconds)
  MSP_SET_COMMAND 217    u16 command (1 = take-off, 2 = land). The firmware
                         reads 2 bytes (read16); the plutocontrol library
                         sends 1, which only works because of leftover
                         buffer contents - this client sends 2.

The Pluto X accepts MSP over TCP: 192.168.4.1:23 on the drone's own Wi-Fi,
192.168.0.1:9060 through the camera module's Wi-Fi.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

MSP_STATUS = 101
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_BOXIDS = 119
MSP_SET_RAW_RC = 200
MSP_SET_COMMAND = 217

COMMAND_TAKE_OFF = 1
COMMAND_LAND = 2

# FlightStatus_e bits of flightIndicatorFlag
FLAG_ACCEL_GYRO_CALIBRATION = 0
FLAG_MAG_CALIBRATION = 1
FLAG_CRASH = 2
FLAG_LOW_BATTERY = 3
FLAG_LOW_BATTERY_IN_FLIGHT = 4
FLAG_SIGNAL_LOSS = 5
FLAG_NOT_OK_TO_ARM = 6
FLAG_OK_TO_ARM = 7
FLAG_ARMED = 8

# permanent box ids (io/rc_controls.h boxId_e order, Cleanflight permanent ids)
BOX_ARM = 0
BOX_ANGLE = 1
BOX_BARO = 3

_HEADER_REQUEST = b'$M<'
_HEADER_REPLY = b'$M>'
_HEADER_ERROR = b'$M!'


def checksum(size: int, cmd: int, payload: bytes) -> int:
    value = size ^ cmd
    for byte in payload:
        value ^= byte
    return value & 0xFF


def encode(cmd: int, payload: bytes = b'', header: bytes = _HEADER_REQUEST) -> bytes:
    """One MSP v1 frame (a request by default)."""
    if not 0 <= cmd <= 255:
        raise ValueError(f'MSP command {cmd} out of range')
    if len(payload) > 255:
        raise ValueError('MSP v1 payload is at most 255 bytes')
    size = len(payload)
    return header + bytes([size, cmd]) + payload + bytes([checksum(size, cmd, payload)])


def encode_set_raw_rc(channels_us: Sequence[int]) -> bytes:
    """MSP_SET_RAW_RC with the channels in AETR1234 order (microseconds)."""
    if not 1 <= len(channels_us) <= 18:
        raise ValueError('1..18 RC channels')
    for value in channels_us:
        if not 0 <= int(value) <= 0xFFFF:
            raise ValueError(f'RC value {value} out of range')
    return encode(MSP_SET_RAW_RC, struct.pack(f'<{len(channels_us)}H', *map(int, channels_us)))


def encode_set_command(command: int) -> bytes:
    return encode(MSP_SET_COMMAND, struct.pack('<H', command))


@dataclass(frozen=True)
class Frame:
    """A parsed MSP frame."""
    cmd: int
    payload: bytes
    is_error: bool = False


class Parser:
    """Incremental MSP v1 frame parser: feed() any byte chunks; complete,
    checksum-valid frames come out. Garbage and corrupted frames are
    skipped (counted in `errors`)."""

    def __init__(self, accept_headers: Tuple[bytes, ...] = (_HEADER_REPLY, _HEADER_ERROR)):
        self._buffer = bytearray()
        self._accept = accept_headers
        self.errors = 0

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer += data
        frames = []
        while True:
            start = self._buffer.find(b'$M')
            if start < 0:
                # keep a trailing '$' that may start the next header
                del self._buffer[:max(len(self._buffer) - 1, 0)]
                return frames
            del self._buffer[:start]
            if len(self._buffer) < 5:
                return frames
            header = bytes(self._buffer[:3])
            if header not in self._accept:
                self.errors += 1
                del self._buffer[:2]
                continue
            size, cmd = self._buffer[3], self._buffer[4]
            if len(self._buffer) < 6 + size:
                return frames
            payload = bytes(self._buffer[5:5 + size])
            if self._buffer[5 + size] != checksum(size, cmd, payload):
                self.errors += 1
                del self._buffer[:2]
                continue
            del self._buffer[:6 + size]
            frames.append(Frame(cmd=cmd, payload=payload, is_error=header == _HEADER_ERROR))


# ---------------------------------------------------------------- decoders

@dataclass(frozen=True)
class Status:
    flags: int            # flightIndicatorFlag
    sensors: int
    mode_flags: int
    profile: int

    def flag(self, bit: int) -> bool:
        return bool(self.flags >> bit & 1)

    @property
    def armed(self) -> bool:
        return self.flag(FLAG_ARMED)

    @property
    def ok_to_arm(self) -> bool:
        return self.flag(FLAG_OK_TO_ARM)


def decode_status(payload: bytes) -> Status:
    if len(payload) < 11:
        raise ValueError(f'MSP_STATUS: {len(payload)} bytes, expected 11')
    flags, _i2c, sensors, modes, profile = struct.unpack('<HHHIB', payload[:11])
    return Status(flags=flags, sensors=sensors, mode_flags=modes, profile=profile)


@dataclass(frozen=True)
class Attitude:
    roll_decideg: int     # + right side down
    pitch_decideg: int    # + NOSE DOWN (firmware convention)
    heading_deg: int      # 0..359


def decode_attitude(payload: bytes) -> Attitude:
    if len(payload) < 6:
        raise ValueError(f'MSP_ATTITUDE: {len(payload)} bytes, expected 6')
    return Attitude(*struct.unpack('<hhh', payload[:6]))


@dataclass(frozen=True)
class Altitude:
    altitude_cm: int
    vario_cm_s: int


def decode_altitude(payload: bytes) -> Altitude:
    if len(payload) < 6:
        raise ValueError(f'MSP_ALTITUDE: {len(payload)} bytes, expected 6')
    return Altitude(*struct.unpack('<ih', payload[:6]))


@dataclass(frozen=True)
class Analog:
    battery_mv: int
    current_raw: int
    mah_drawn: int
    mah_remaining: int
    state_of_charge_pct: int
    battery_warning: int


def decode_analog(payload: bytes) -> Analog:
    if len(payload) < 10:
        raise ValueError(f'MSP_ANALOG: {len(payload)} bytes, expected 10')
    return Analog(*struct.unpack('<HHHHBB', payload[:10]))


def decode_boxids(payload: bytes) -> List[int]:
    return list(payload)


# -------------------------------------------------------------- connection

class Connection:
    """TCP MSP link with a background reader. Thread-safe send()."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0,
                 on_frame: Optional[Callable[[Frame], None]] = None):
        self._socket = socket.create_connection((host, port), timeout=timeout_s)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._socket.settimeout(0.5)
        self._send_lock = threading.Lock()
        self._parser = Parser()
        self._on_frame = on_frame
        self._latest: Dict[int, Tuple[float, Frame]] = {}
        self._latest_lock = threading.Lock()
        self._closed = threading.Event()
        self.bytes_received = 0
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def send(self, frame_bytes: bytes) -> None:
        with self._send_lock:
            self._socket.sendall(frame_bytes)

    def request(self, cmd: int) -> None:
        """Sends an (empty) request; the reply arrives asynchronously."""
        self.send(encode(cmd))

    def latest(self, cmd: int) -> Optional[Tuple[float, Frame]]:
        """(monotonic time received, frame) of the latest reply to cmd."""
        with self._latest_lock:
            return self._latest.get(cmd)

    def wait_for(self, cmd: int, newer_than: float, timeout_s: float) -> Optional[Frame]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._closed.is_set():
            entry = self.latest(cmd)
            if entry and entry[0] > newer_than:
                return entry[1]
            time.sleep(0.005)
        return None

    @property
    def parse_errors(self) -> int:
        return self._parser.errors

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def _read_loop(self):
        while not self._closed.is_set():
            try:
                data = self._socket.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            self.bytes_received += len(data)
            now = time.monotonic()
            for frame in self._parser.feed(data):
                with self._latest_lock:
                    self._latest[frame.cmd] = (now, frame)
                if self._on_frame:
                    self._on_frame(frame)
        self._closed.set()

    def close(self) -> None:
        self._closed.set()
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()
        self._reader.join(timeout=1.0)
