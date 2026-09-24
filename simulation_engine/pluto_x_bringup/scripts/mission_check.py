#!/usr/bin/env python3
"""End-to-end check of an outer-loop waypoint mission flown through the
MagisV2 firmware (run by scripts/check_mission.sh next to sim.launch.py).

Records ground truth (/sim/pluto/odometry) and the flight-controller
status (/pluto/fc_status) until the vehicle has been armed and is disarmed
again (or a timeout), then checks:
  * the flight controller armed and later disarmed (mission completed)
  * every waypoint was passed within --tolerance metres
  * (informational only) where the vehicle touched down: the competition
    task has no landing requirement
  * the tilt never exceeded --max-tilt-deg
  * altitude never exceeded the highest waypoint by more than 0.5 m
  * tracking error (true position - controller setpoint, from
    /pluto/outer_loop/setpoint): reported overall (includes the transient
    after each setpoint step) and settled (samples more than --settle-s
    after a setpoint change); the settled mean must be < --max-hold-error

Usage: mission_check.py <waypoint params yaml> [--timeout S] [--csv FILE]
"""

import argparse
import math
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

from pluto_x_interfaces.msg import FlightControllerStatus, OuterLoopSetpoint


def tilt_rad(q) -> float:
    # angle between body z and world z: cos = 1 - 2 (x^2 + y^2)
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))))


class Recorder(Node):

    def __init__(self):
        super().__init__('mission_check')
        self.samples = []  # (t, x, y, z, tilt)
        self.setpoints = []  # (t, x, y, z)
        self.armed_seen = False
        self.disarmed_after_arm = False
        self.create_subscription(Odometry, '/sim/pluto/odometry', self.on_odom, 50)
        self.create_subscription(FlightControllerStatus, '/pluto/fc_status',
                                 self.on_status, 50)
        self.create_subscription(OuterLoopSetpoint, '/pluto/outer_loop/setpoint',
                                 self.on_setpoint, 50)

    def on_setpoint(self, msg):
        if not msg.has_position:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.setpoints.append((t, msg.x_enu_m, msg.y_enu_m, msg.z_enu_m))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((t, p.x, p.y, p.z, tilt_rad(msg.pose.pose.orientation)))

    def on_status(self, msg):
        if msg.armed:
            self.armed_seen = True
        elif self.armed_seen:
            self.disarmed_after_arm = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('params')
    parser.add_argument('--timeout', type=float, default=240.0,
                        help='wall-clock seconds')
    parser.add_argument('--tolerance', type=float, default=0.3)
    parser.add_argument('--max-tilt-deg', type=float, default=25.0)
    parser.add_argument('--csv', default='')
    parser.add_argument('--settle-s', type=float, default=3.0)
    parser.add_argument('--max-hold-error', type=float, default=0.2)
    args = parser.parse_args()
    with open(args.params, 'r', encoding='utf-8') as stream:
        waypoints = [np.asarray(w, dtype=float)
                     for w in yaml.safe_load(stream)['waypoints_enu_m']]

    rclpy.init()
    node = Recorder()
    clock = node.get_clock()  # wall clock for the timeout
    start = clock.now()
    while rclpy.ok() and not node.disarmed_after_arm:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (clock.now() - start).nanoseconds * 1e-9 > args.timeout:
            break
    for _ in range(20):  # a little settling data after disarm
        rclpy.spin_once(node, timeout_sec=0.05)
    samples = np.array(node.samples) if node.samples else np.zeros((0, 5))
    setpoints = np.array(node.setpoints) if node.setpoints else np.zeros((0, 4))
    armed, disarmed = node.armed_seen, node.disarmed_after_arm
    node.destroy_node()
    rclpy.shutdown()

    if args.csv and len(samples):
        np.savetxt(args.csv, samples, delimiter=',',
                   header='t_s,x_enu_m,y_enu_m,z_enu_m,tilt_rad', comments='')
    failures = 0

    def check(ok, text):
        nonlocal failures
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
        failures += 0 if ok else 1

    print(f'mission: {len(samples)} odometry samples'
          + (f', t = {samples[0, 0]:.2f} .. {samples[-1, 0]:.2f} s'
             if len(samples) else ''))
    check(armed and disarmed, f'armed ({armed}) and disarmed after the mission '
          f'({disarmed})')
    if len(samples) == 0:
        print('RESULT: FAIL (no data)')
        return 1
    origin = samples[0, 1:4]
    track = samples[:, 1:4] - origin
    for i, waypoint in enumerate(waypoints):
        closest = float(np.min(np.linalg.norm(track - waypoint, axis=1)))
        check(closest < args.tolerance,
              f'waypoint {i} {waypoint.tolist()}: closest approach '
              f'{closest:.3f} m < {args.tolerance} m')
    final = track[-1]
    print(f'  [INFO] touched down at ({final[0]:.2f}, {final[1]:.2f}, '
          f'{final[2]:.2f}) m from take-off (not a mission requirement)')
    max_tilt = math.degrees(float(np.max(samples[:, 4])))
    check(max_tilt < args.max_tilt_deg,
          f'max tilt {max_tilt:.1f} deg < {args.max_tilt_deg} deg')
    ceiling = max(w[2] for w in waypoints) + 0.5
    check(float(np.max(track[:, 2])) < ceiling,
          f'max altitude {np.max(track[:, 2]):.2f} m < {ceiling:.2f} m')
    failures += report_tracking(samples, setpoints, args, check)
    print('RESULT: ' + ('PASS' if failures == 0 else f'FAIL ({failures} check(s))'))
    return 0 if failures == 0 else 1


def report_tracking(samples, setpoints, args, check) -> int:
    """Error of the true position against the controller setpoint."""
    if len(setpoints) == 0:
        check(False, 'no setpoints received on /pluto/outer_loop/setpoint')
        return 0
    # odometry samples inside the setpoint period, paired with the latest
    # setpoint at or before them (sample-and-hold)
    inside = (samples[:, 0] >= setpoints[0, 0]) & (samples[:, 0] <= setpoints[-1, 0])
    odo = samples[inside]
    index = np.searchsorted(setpoints[:, 0], odo[:, 0], side='right') - 1
    target = setpoints[index, 1:4]
    error = odo[:, 1:4] - target
    e3 = np.linalg.norm(error, axis=1)
    eh = np.linalg.norm(error[:, :2], axis=1)
    ev = np.abs(error[:, 2])
    # time since the setpoint last changed
    changed = np.r_[True, np.any(np.diff(setpoints[:, 1:4], axis=0) != 0.0, axis=1)]
    change_times = setpoints[changed, 0]
    last_change = change_times[np.searchsorted(change_times, odo[:, 0], side='right') - 1]
    settled = odo[:, 0] - last_change > args.settle_s

    def stats(values):
        return (f'mean {np.mean(values):.3f}  rms {np.sqrt(np.mean(values ** 2)):.3f}'
                f'  max {np.max(values):.3f} m')

    print(f'tracking error, true - setpoint ({len(odo)} samples, '
          f'{len(change_times)} setpoints):')
    print(f'  overall   3D {stats(e3)}')
    print(f'            horizontal {stats(eh)} | vertical {stats(ev)}')
    if not np.any(settled):
        check(False, 'no settled samples to evaluate')
        return 0
    print(f'  settled   3D {stats(e3[settled])}   (> {args.settle_s:g} s after a setpoint change)')
    print(f'            horizontal {stats(eh[settled])} | vertical {stats(ev[settled])}')
    print('  per setpoint (settled):')
    segment = np.searchsorted(change_times, odo[:, 0], side='right') - 1
    for i, t0 in enumerate(change_times):
        mask = settled & (segment == i)
        sp = setpoints[changed][i, 1:4]
        if np.any(mask):
            print(f'    {i}: target ({sp[0]:+.2f}, {sp[1]:+.2f}, {sp[2]:+.2f}) m  '
                  f'from t={t0:6.2f} s  mean {np.mean(e3[mask]):.3f} m  '
                  f'max {np.max(e3[mask]):.3f} m  (n={int(np.sum(mask))})')
        else:
            print(f'    {i}: target ({sp[0]:+.2f}, {sp[1]:+.2f}, {sp[2]:+.2f}) m  '
                  f'from t={t0:6.2f} s  (left before settling)')
    mean_settled = float(np.mean(e3[settled]))
    check(mean_settled < args.max_hold_error,
          f'settled mean tracking error {mean_settled:.3f} m < {args.max_hold_error} m')
    return 0  # counted through check()


if __name__ == '__main__':
    sys.exit(main())
