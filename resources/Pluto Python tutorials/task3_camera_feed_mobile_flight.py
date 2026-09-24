"""
Task 3 - Show the live camera feed in Python while flying from the Pluto app
============================================================================
HARDWARE : Pluto drone WITH the camera module installed + a phone
LIBRARY  : plutocam                 ->  pip install plutocam
TOOL     : ffplay  (part of ffmpeg) ->  see install note below
           (NOTE: plutocontrol is NOT used here - the phone flies the drone)

THE SETUP  (two radios, two jobs)
---------------------------------
    COMPUTER  --joins-->  CAMERA module Wi-Fi   ->  receives + shows the video
    PHONE     --joins-->  DRONE built-in Wi-Fi  ->  flies with the Pluto app

So you can watch the live video on the laptop while a second person (or you)
flies the drone manually from the phone. This script ONLY shows the feed.

NO OPENCV NEEDED
----------------
The camera sends a raw H.264 stream. We pipe those bytes straight to
**ffplay**, which opens its own video window - no decoding code required.

INSTALL ffplay / ffmpeg
-----------------------
    * Mac     : brew install ffmpeg
    * Windows : download from https://ffmpeg.org/download.html (add to PATH)
    * Linux   : sudo apt install ffmpeg

HOW TO RUN
----------
  1. Fit the camera module and switch the drone to Camera Wi-Fi mode.
  2. On the PHONE: join the drone's built-in Wi-Fi and open the Pluto app.
  3. On the COMPUTER: join the CAMERA module's Wi-Fi.
  4. pip install plutocam     (and install ffmpeg, see above)
  5. python3 task3_camera_feed_mobile_flight.py

  Close the video window or press Ctrl+C in the terminal to stop.

TIP: want to SAVE the feed to a file instead of (or as well as) watching it?
     ffmpeg can record the same stream - ask and we can add a record option.
"""

import shutil
import subprocess

from plutocam import LWDrone


def main():
    # ffplay must be installed (it ships with ffmpeg).
    if shutil.which("ffplay") is None:
        print("[ERROR] ffplay not found. Install ffmpeg first:")
        print("        Mac: brew install ffmpeg | Windows/Linux: see file header.")
        return

    cam = LWDrone()            # default host 192.168.0.1

    # ffplay reads raw H.264 from its standard input and shows it in a window.
    player = subprocess.Popen(
        [
            "ffplay",
            "-loglevel", "error",
            "-fflags", "nobuffer",     # low-latency playback
            "-flags", "low_delay",
            "-framedrop",
            "-window_title", "Pluto Camera Feed (phone is flying)",
            "-f", "h264", "-i", "-",   # input = raw H.264, from stdin
        ],
        stdin=subprocess.PIPE,
    )

    print("[INFO] Showing the live camera feed. Close the window or Ctrl+C to stop.")

    try:
        for frame in cam.start_video_stream():
            if player.stdin is None:
                break
            player.stdin.write(frame.frame_bytes)
    except (KeyboardInterrupt, BrokenPipeError, ValueError, OSError):
        pass
    finally:
        cam.stop_video_stream()
        try:
            if player.stdin is not None:
                player.stdin.close()
        except Exception:
            pass
        player.terminate()
        print("[OK] Stopped.")


if __name__ == "__main__":
    main()
