"""
Task 2 - Control the drone while watching the live camera feed (Camera Wi-Fi)
=============================================================================
HARDWARE : Pluto drone WITH the camera module installed
LIBRARIES: plutocontrol, plutocam          ->  pip install plutocontrol plutocam
TOOL     : ffplay  (part of ffmpeg)        ->  see install note below

NO OPENCV NEEDED
----------------
The camera sends a raw H.264 video stream. Instead of decoding it ourselves,
we simply pipe those bytes to **ffplay**, which opens its own video window.
So this script does two things at once on the Camera Wi-Fi:

    1. SHOWS the live video           (plutocam  -> ffplay window)
    2. FLIES the drone with the keys  (plutocontrol over 192.168.0.1:9060)

INSTANT KEYS (no Enter!)
------------------------
Like Task 1, this reads **single key presses** in real time - just tap a key,
no need to press Enter. Keep the TERMINAL window focused while you fly; the
ffplay window just shows the video.

    space -> take off (arms first)     l -> land
    w / s -> forward / backward        x -> stop (re-centre sticks)
    a / d -> left / right              0 -> disarm (emergency stop)
    j / k -> yaw left / right          q -> quit (lands, then exits)
    r / f -> up / down

INSTALL ffplay / ffmpeg
-----------------------
    * Mac     : brew install ffmpeg
    * Windows : download from https://ffmpeg.org/download.html (add to PATH)
    * Linux   : sudo apt install ffmpeg

HOW TO RUN
----------
  1. Press the onboard ESP button to disable the drone's built-in Wi-Fi.
  2. Switch the drone to Camera Wi-Fi mode and join its Wi-Fi (PlutoCam_...).
  3. pip install plutocontrol plutocam   (and install ffmpeg, see above)
  4. python3 task2_camera_wifi_vision.py

NOTE: the real-time key capture uses the Unix terminal (macOS / Linux).
      On Windows, run it from WSL or a Unix-like terminal.

SAFETY
------
  * A movement key holds until you change it - press 'x' to stop (re-centre).
  * '0' disarms instantly (emergency stop); 'q' lands and disarms on exit.
  * Fly in a large open area, never near faces or people.
"""

import shutil
import subprocess
import sys
import termios
import threading
import tty

from plutocam import LWDrone
from plutocontrol import Pluto


# ---------------------------------------------------------------------------
# Read a single key press without waiting for Enter (Unix: macOS / Linux).
# Raw mode is enabled only for the one read, then the terminal is restored,
# so normal print() output still looks fine between key presses.
# ---------------------------------------------------------------------------
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


# ---------------------------------------------------------------------------
# Video: pipe the raw H.264 frames from the camera straight into ffplay.
# Runs in a background thread so the keyboard stays responsive.
# ---------------------------------------------------------------------------
def start_video_player():
    """Launch ffplay reading H.264 from stdin; return the process or None."""
    if shutil.which("ffplay") is None:
        print("[WARN] ffplay not found - the feed will not be shown.")
        print("       Install ffmpeg (see the note at the top of this file).")
        return None
    return subprocess.Popen(
        [
            "ffplay",
            "-loglevel", "error",      # keep the terminal clean for the keys
            "-fflags", "nobuffer",     # low-latency flags
            "-flags", "low_delay",
            "-framedrop",
            "-window_title", "Task 2 - Pluto Camera Feed",
            "-f", "h264", "-i", "-",   # input format = raw H.264, from stdin
        ],
        stdin=subprocess.PIPE,
    )


def video_loop(cam, player, stop_event):
    """Read frames from the camera and feed them to ffplay until stopped."""
    try:
        for frame in cam.start_video_stream():
            if stop_event.is_set():
                break
            try:
                player.stdin.write(frame.frame_bytes)
            except (BrokenPipeError, ValueError, OSError):
                break   # ffplay window was closed
    except Exception:
        pass


def main():
    # --- 1. CONTROL LINK over the camera module --------------------------
    pluto = Pluto()
    pluto.cam()            # MUST be before connect(): retargets to 192.168.0.1:9060
    pluto.connect()
    if not pluto.connected:
        print("[ERROR] No control link on the camera Wi-Fi (192.168.0.1:9060).")
        print("        Check you joined the camera module's network.")
        return

    # --- 2. VIDEO WINDOW (ffplay) in a background thread -----------------
    cam = LWDrone()                 # default host 192.168.0.1
    player = start_video_player()
    stop_event = threading.Event()
    if player is not None:
        threading.Thread(
            target=video_loop, args=(cam, player, stop_event), daemon=True
        ).start()

    # --- 3. FLY WITH SINGLE KEY PRESSES ---------------------------------
    print("Ready. Focus this terminal and fly:")
    print("  space=take off  w/a/s/d=move  j/k=yaw  r/f=up/down")
    print("  x=stop  l=land  0=disarm  q=quit")

    armed = False
    try:
        while True:
            key = get_key().lower()

            if key == " ":
                print("Take off")
                pluto.arm()
                pluto.take_off()
                armed = True
            elif key == "w":
                print("Forward"); pluto.forward()
            elif key == "s":
                print("Backward"); pluto.backward()
            elif key == "a":
                print("Left"); pluto.left()
            elif key == "d":
                print("Right"); pluto.right()
            elif key == "j":
                print("Yaw left"); pluto.left_yaw()
            elif key == "k":
                print("Yaw right"); pluto.right_yaw()
            elif key == "r":
                print("Up"); pluto.increase_height()
            elif key == "f":
                print("Down"); pluto.decrease_height()
            elif key == "x":
                print("Stop"); pluto.reset()
            elif key == "l":
                print("Land"); pluto.land()
            elif key == "0":
                print("Disarm"); pluto.disarm(); armed = False
            elif key == "q":
                print("Quit")
                pluto.land()
                pluto.disarm()
                armed = False
                break
            # any other key is ignored

    except KeyboardInterrupt:
        pass
    finally:
        # stop everything cleanly
        stop_event.set()
        cam.stop_video_stream()
        if armed:
            pluto.disarm()
        pluto.disconnect()
        if player is not None:
            try:
                if player.stdin is not None:
                    player.stdin.close()
            except Exception:
                pass
            player.terminate()
        print("[OK] Disarmed, disconnected, video stopped.")


if __name__ == "__main__":
    main()
