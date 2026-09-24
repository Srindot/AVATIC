#!/usr/bin/env python3
"""YOUR OUTER-LOOP CONTROLLER: write your balloon-popping algorithm here.

Run the simulator in one terminal:
    ros2 launch pluto_x_bringup competition.launch.py
and this file in another (both with the workspace sourced):
    python3 outerloop_controller/my_controller.py

The rules: pop green (+100), blue (+50) and yellow (+25) balloons by
touching them; red balloons are -75 (avoid them). You have 15 s from the
moment you arm. The balloon positions change every run: you have to find
them with the camera.

You get:   drone.get_frame()      camera image (numpy RGB, 1280 x 720, ~18 fps)
           drone.get_telemetry()  attitude, heading, baro altitude, battery
           drone.arena()          score, time left
You send:  drone.send_command(roll, pitch, yaw_rate, throttle)
           (see avatic_drone/drone.py for what each value means)
"""

from avatic_drone import Drone

CONTROL_RATE_HZ = 20
HOVER_THROTTLE = 0.76   # about hover in the simulator; fine-tune with feedback


def main():
    drone = Drone()
    drone.wait_until_ready()
    drone.arm()
    try:
        for _ in drone.loop(hz=CONTROL_RATE_HZ):
            frame = drone.get_frame()
            telemetry = drone.get_telemetry()

            # ---------------------------------------------------------
            # TODO: your algorithm. Find balloons in frame.image, decide
            # where to go, and turn that into stick commands.
            # ---------------------------------------------------------
            roll, pitch, yaw_rate, throttle = 0.0, 0.0, 0.0, HOVER_THROTTLE

            drone.send_command(roll=roll, pitch=pitch, yaw_rate=yaw_rate,
                               throttle=throttle)
    finally:
        print(f'score: {drone.arena().score}')
        drone.close()


if __name__ == '__main__':
    main()
