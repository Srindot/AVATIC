#!/usr/bin/env python3
"""End-to-end flight checks for the legacy-stack Pluto X simulation.

Run against a simulation launched PAUSED (legacy_sim.launch.py paused:=true);
the check unpauses the world once its subscriptions are ready so the whole
flight from t = 0 is recorded.

Scenario
--------
hold      Expects the sim to start in position_hold (pilot.initial_mode).
          Records ground truth and checks convergence to the target.

Outputs a CSV (t_s, x_enu_m, y_enu_m, z_enu_m, roll_frd_rad, pitch_frd_rad,
yaw_frd_rad) of the ground-truth trajectory and exits non-zero if any check
fails. Ground truth comes from /sim/pluto/odometry, which is simulator-only.
"""

import argparse
import csv
import math
import subprocess
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def frd_euler_from_enu_flu_quaternion(w, x, y, z):
    """Z-Y-X Euler angles of the FRD body w.r.t. NED for an ENU/FLU pose.

    Same convention as pluto_x::frames (R_ned_frd = S_w R_enu_flu S_b).
    """
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    s_w = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    s_b = [1, -1, -1]
    ned = [[sum(s_w[i][k] * r[k][j] for k in range(3)) * s_b[j]
            for j in range(3)] for i in range(3)]
    pitch = math.asin(max(-1.0, min(1.0, -ned[2][0])))
    roll = math.atan2(ned[2][1], ned[2][2])
    yaw = math.atan2(ned[1][0], ned[0][0])
    return roll, pitch, yaw


class FlightCheck(Node):
    def __init__(self, scenario, duration_s, world):
        super().__init__('legacy_flight_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._scenario = scenario
        self._duration_s = duration_s
        self._world = world
        self.samples = []  # (t, x, y, z, roll, pitch, yaw)
        self._odom_sub = self.create_subscription(
            Odometry, '/sim/pluto/odometry', self._on_odometry, 100)

    def ready(self):
        return self.count_publishers('/sim/pluto/odometry') > 0

    def unpause(self):
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self._world}/control',
             '--reqtype', 'gz.msgs.WorldControl', '--reptype',
             'gz.msgs.Boolean', '--timeout', '5000', '--req', 'pause: false'],
            check=True, capture_output=True)

    def done(self):
        return bool(self.samples) and self.samples[-1][0] >= self._duration_s

    def _on_odometry(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = frd_euler_from_enu_flu_quaternion(q.w, q.x, q.y, q.z)
        self.samples.append((t, p.x, p.y, p.z, roll, pitch, yaw))


class Checks:
    def __init__(self):
        self.failures = 0

    def expect(self, condition, description):
        print(f'  [{"PASS" if condition else "FAIL"}] {description}')
        if not condition:
            self.failures += 1


# Hold statistics are taken over the final window of the run.
HOLD_WINDOW_S = 10.0
# Tilt allowed in the final window (sensor bias and gusts tilt the vehicle).
MAX_SETTLED_TILT_RAD = 0.1


def evaluate_hold(samples, target, tolerance_m, checks):
    t_end = samples[-1][0]
    window = [s for s in samples if s[0] >= t_end - HOLD_WINDOW_S]
    mean_error = sum(math.dist(s[1:4], target) for s in window) / len(window)
    max_error = max(math.dist(s[1:4], target) for s in window)
    max_alt = max(s[3] for s in samples)
    max_tilt = max(max(abs(s[4]), abs(s[5])) for s in window)
    checks.expect(mean_error < tolerance_m,
                  f'mean distance to target over last {HOLD_WINDOW_S:.0f} s '
                  f'{mean_error:.4f} m < {tolerance_m} m (max {max_error:.4f} m)')
    checks.expect(max_alt < 3.0 * target[2],
                  f'bounded altitude (max {max_alt:.3f} m)')
    checks.expect(max_tilt < MAX_SETTLED_TILT_RAD,
                  f'settled attitude (max tilt {max_tilt:.4f} rad over last '
                  f'{HOLD_WINDOW_S:.0f} s)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('scenario', choices=['hold'])
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output', default='flight.csv')
    parser.add_argument('--target', type=float, nargs=3,
                        default=[1.0, 1.0, 1.0])
    parser.add_argument('--tolerance', type=float, default=0.05)
    parser.add_argument('--world', default='pluto_legacy')
    args = parser.parse_args()

    rclpy.init()
    node = FlightCheck(args.scenario, args.duration, args.world)
    while rclpy.ok() and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.unpause()
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    with open(args.output, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s', 'x_enu_m', 'y_enu_m', 'z_enu_m',
                         'roll_frd_rad', 'pitch_frd_rad', 'yaw_frd_rad'])
        writer.writerows(samples)

    print(f'{args.scenario}: {len(samples)} samples, '
          f't = {samples[0][0]:.3f} .. {samples[-1][0]:.3f} s -> {args.output}')
    checks = Checks()
    evaluate_hold(samples, args.target, args.tolerance, checks)
    print('RESULT:', 'PASS' if checks.failures == 0 else
          f'FAIL ({checks.failures} check(s))')
    return 0 if checks.failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
