#!/usr/bin/env python3
"""Example: take off, hold ~1 m on the barometer, turn slowly and report
how much of each balloon colour the camera sees. Shows every part of the
interface; it does NOT pop balloons (that is your job).

    python3 outerloop_controller/examples/hello_drone.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from avatic_drone import Drone  # noqa: E402

TARGET_ALTITUDE_M = 1.0
HOVER_THROTTLE = 0.76
ALTITUDE_GAIN = 0.15      # throttle per metre of altitude error
CLIMB_DAMPING = 0.10      # throttle per m/s (from the altitude change)

# crude colour masks (RGB), just to show that the image is usable
COLOURS = {
    'green': lambda r, g, b: (g > 90) & (r < 50) & (b < 60),
    'blue': lambda r, g, b: (b > 120) & (r < 60) & (g < 110),
    'yellow': lambda r, g, b: (r > 150) & (g > 120) & (b < 70),
    'red': lambda r, g, b: (r > 120) & (g < 60) & (b < 60),
}


def colour_pixels(image):
    r, g, b = (image[..., i].astype(int) for i in range(3))
    return {name: int(np.count_nonzero(mask(r, g, b))) for name, mask in COLOURS.items()}


def main():
    drone = Drone()
    try:
        print('connected; waiting for the flight controller...')
        drone.wait_until_ready()
        drone.arm()
        start = drone.time()
        print(f'armed at simulation time {start:.2f} s; t below = seconds since arming')
        last_altitude, last_t = drone.get_telemetry().altitude_m, 0.0
        for step in drone.loop(hz=20):
            t = drone.time() - start
            tel = drone.get_telemetry()
            dt = t - last_t
            climb_rate = (tel.altitude_m - last_altitude) / dt if dt > 0 else 0.0
            last_altitude, last_t = tel.altitude_m, t
            throttle = (HOVER_THROTTLE + ALTITUDE_GAIN * (TARGET_ALTITUDE_M - tel.altitude_m)
                        - CLIMB_DAMPING * climb_rate)
            # climb straight up first; start turning once near 1 m
            yaw_rate = 0.3 if tel.altitude_m > 0.7 else 0.0
            drone.send_command(roll=0.0, pitch=0.0, yaw_rate=yaw_rate, throttle=throttle)
            if step % 20 == 0:
                frame = drone.get_frame()
                seen = colour_pixels(frame.image) if frame else {}
                print(f't={t:5.1f} s  alt(baro)={tel.altitude_m:4.2f} m  heading={tel.heading_deg:5.1f}  '
                      f'score={drone.arena().score}  seen={seen}')
    finally:
        print(f'final score: {drone.arena().score}')
        drone.close()


if __name__ == '__main__':
    main()
