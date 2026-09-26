#!/usr/bin/env python3
"""YOUR OUTER-LOOP CONTROLLER - write your balloon-popping algorithm here.

New here? Read docs/participants/README.md (the participant guide).

Run it (simulator + this file, one command):
    ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
or in two terminals:
    ros2 launch pluto_x_bringup competition.launch.py
    python3 outerloop_controller/my_controller.py
On the REAL Pluto X (join the camera module's Wi-Fi first):
    python3 outerloop_controller/my_controller.py --hardware

Task: pop green (+100), blue (+50) and yellow (+25) balloons by touching
them; red balloons are -75 (avoid them). 25 s from the moment the drone
arms. Each simulator launch places the balloons from a seed: by default
the fixed development seed in analysis/seed.yaml (the same layout every
run, for tuning); `arena_seed:=random` gives a new layout, and
evaluation/evaluate.py tests on many random layouts.

WHAT YOU WRITE: the MyController class below (section 2). Every control
step the template calls

    command = controller.step(frame, telemetry, t)

  frame      the latest camera image, or None before the first one
               frame.image   numpy array 720 x 1280 x 3, uint8, RGB
               frame.seq     frame number (same seq = same image as before)
               frame.time_s  when it was taken
  telemetry  the flight controller's estimates
               .altitude_m (barometric), .heading_deg, .roll_deg,
               .pitch_deg, .battery_v, .armed
  t          seconds since arming (the run clock)

and step() returns a Command:

    Command(roll=..., pitch=..., yaw_rate=..., throttle=...)
      roll      -1..1  bank angle     + = bank right   (~7 deg at 0.2, ~16 at 0.4,
                                                        20 deg max from ~0.45)
      pitch     -1..1  tilt angle     + = nose down = fly forward
      yaw_rate  -1..1  turn rate      + = clockwise    (~77 deg/s per unit,
                                                        capped at 0.8)
      throttle   0..1  thrust         ~0.76 = hover

The template sends it to the drone, which is all the ROS you need.
"""

import argparse
import os
import traceback

import numpy as np  # noqa: F401  (you will probably need it)

from avatic_drone import Command, Drone

# =============================================================================
# 1. SETTINGS (change freely)
# =============================================================================
CONTROL_RATE_HZ = 20          # how often step() is called (camera: ~18 fps)
HOVER_THROTTLE = 0.76         # throttle that roughly holds altitude
TAKEOFF_ALTITUDE_M = 1.0      # the example below climbs to this and hovers


# =============================================================================
# 2. YOUR CONTROLLER - WRITE YOUR CODE HERE
# =============================================================================
class MyController:

    def __init__(self):
        # Set up anything you need: detector parameters, filters, state...
        self.last_seq = -1
        self.last_altitude_m = None   # set from the first telemetry
        self.last_t = 0.0

    def step(self, frame, telemetry, t) -> Command:
        """Called every control step. Return what the drone should do now."""

        # -- 2a. Look at the camera -------------------------------------------
        if frame is not None and frame.seq != self.last_seq:
            self.last_seq = frame.seq
            image = frame.image  # noqa: F841 (720 x 1280 x 3 RGB)
            # TODO: find balloons in `image` (colour, position in the image,
            #       apparent size) and decide which one to go for.

        # -- 2b. Decide the command -------------------------------------------
        # Example (replace it): climb to TAKEOFF_ALTITUDE_M and hover in
        # place, using the barometric altitude. Take off straight up before
        # you tilt or turn.
        if self.last_altitude_m is None:
            self.last_altitude_m = telemetry.altitude_m
        dt = t - self.last_t
        climb_rate = (telemetry.altitude_m - self.last_altitude_m) / dt if dt > 0 else 0.0
        self.last_altitude_m, self.last_t = telemetry.altitude_m, t
        throttle = (HOVER_THROTTLE
                    + 0.15 * (TAKEOFF_ALTITUDE_M - telemetry.altitude_m)
                    - 0.10 * climb_rate)

        # TODO: set yaw_rate to turn towards a balloon, pitch to fly at it,
        #       roll to correct sideways - and stay away from red ones.
        return Command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=throttle)


# =============================================================================
# 3. RUNNER - no need to change anything below this line
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description='Run the outer-loop controller.')
    parser.add_argument('--hardware', action='store_true',
                        help='fly the real Pluto X instead of the simulator')
    parser.add_argument('--host', help='(hardware) Pluto X address (default 192.168.0.1)')
    parser.add_argument('--msp-port', type=int, help='(hardware) MSP TCP port (default 9060)')
    parser.add_argument('--video',
                        help="(hardware) 'plutocam' (default), 'tcp://host:port' or 'none'")
    args = parser.parse_args()
    # options not given here fall back to AVATIC_BACKEND / AVATIC_HOST /
    # AVATIC_MSP_PORT / AVATIC_VIDEO, then to the defaults
    options = {k: v for k, v in (('host', args.host), ('msp_port', args.msp_port),
                                 ('video', args.video)) if v is not None}
    if options.get('video') == 'none':
        options['video'] = None
    backend = 'hardware' if args.hardware else os.environ.get('AVATIC_BACKEND', 'sim')
    drone = Drone(backend=backend, **(options if backend == 'hardware' else {}))
    try:
        print('[controller] connected; waiting for the flight controller...', flush=True)
        drone.wait_until_ready()
        controller = MyController()
        drone.arm()
        start = drone.time()
        print(f'[controller] armed at t = {start:.2f} s - the run clock is running',
              flush=True)
        for _ in drone.loop(hz=CONTROL_RATE_HZ):
            try:
                command = controller.step(drone.get_frame(), drone.get_telemetry(),
                                          drone.time() - start)
                drone.send(command)
            except Exception:  # noqa: BLE001 - show the participant their error
                traceback.print_exc()
                print('[controller] step() raised an error (above): commands stopped; '
                      'the failsafe levels the drone at about hover throttle (it may slowly sink).', flush=True)
                while drone.running():  # no more commands: the failsafe takes over
                    drone.sleep(0.5)
                break
    except KeyboardInterrupt:
        print('[controller] interrupted', flush=True)
    finally:
        if backend == 'hardware':
            print('[controller] run over (on the real drone the judges count the balloons)',
                  flush=True)
        else:
            print(f'[controller] score: {drone.arena().score}', flush=True)
        drone.close()


if __name__ == '__main__':
    main()
