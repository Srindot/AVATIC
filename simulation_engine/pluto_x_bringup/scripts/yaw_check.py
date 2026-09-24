#!/usr/bin/env python3
"""Yaw-axis check of the vehicle + MagisV2 + outer loop (run by
scripts/check_yaw.sh next to sim.launch.py).

Records, until the vehicle is disarmed after flying (or a timeout):
  /sim/pluto/odometry          true position, yaw (ENU), body yaw rate, tilt
  /pluto/fc_status             firmware heading estimate
  /pluto/rc                    yaw stick
  /pluto/outer_loop/setpoint   yaw setpoint (closed-loop mode)

Mode open_loop (examples/yaw_open_loop.py: yaw-stick steps while holding
position) reports per step: steady yaw rate and rate per unit stick, 10-90 %
rise time, overshoot, rate ripple (limit-cycle check), position and altitude
disturbance, firmware heading error. Checks: rate sign (+ stick = clockwise),
rate proportional to stick, no sustained oscillation, bounded coupling.

Mode closed_loop (WaypointController with headings) reports yaw tracking
error overall and settled, per heading setpoint, the position drift during
the turns, and the firmware heading error. Checks: every heading reached,
settled error small, no overshoot beyond a bound, position held.

Usage: yaw_check.py {open_loop|closed_loop} [--timeout S] [--out DIR]
"""

import argparse
import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from pluto_x_interfaces.msg import (FlightControllerStatus, OuterLoopSetpoint,
                                    RcCommand)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):

    def __init__(self):
        super().__init__('yaw_check')
        self.odom = []    # t, x, y, z, yaw_enu, r_flu, tilt
        self.status = []  # t, heading_deg (firmware, CW from north), armed
        self.rc = []      # t (sim time stamp), yaw_us
        self.sp = []      # t, yaw_rad
        self.armed_seen = False
        self.done = False
        self.create_subscription(Odometry, '/sim/pluto/odometry', self.on_odom, 100)
        self.create_subscription(FlightControllerStatus, '/pluto/fc_status',
                                 self.on_status, 100)
        self.create_subscription(RcCommand, '/pluto/rc', self.on_rc, 100)
        self.create_subscription(OuterLoopSetpoint, '/pluto/outer_loop/setpoint',
                                 self.on_sp, 100)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))
        p = m.pose.pose.position
        self.odom.append((stamp_s(m.header.stamp), p.x, p.y, p.z, yaw,
                          m.twist.twist.angular.z, tilt))

    def on_status(self, m):
        self.status.append((stamp_s(m.header.stamp), m.heading_deg, float(m.armed)))
        if m.armed:
            self.armed_seen = True
        elif self.armed_seen:
            self.done = True

    def on_rc(self, m):
        self.rc.append((stamp_s(m.header.stamp), m.yaw_us))

    def on_sp(self, m):
        if m.has_yaw:
            self.sp.append((stamp_s(m.header.stamp), m.yaw_rad))


FAILURES = 0


def check(ok, text):
    global FAILURES
    print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
    FAILURES += 0 if ok else 1


def unwrap(yaw):
    return np.unwrap(yaw)


def hold(t_query, t, values):
    """Sample-and-hold lookup of values(t) at t_query."""
    i = np.clip(np.searchsorted(t, t_query, side='right') - 1, 0, len(t) - 1)
    return values[i]


def heading_error_deg(odom, status):
    """Firmware heading (CW from north) - true heading, wrapped, deg."""
    t_s = status[:, 0]
    true_heading = (90.0 - np.degrees(hold(t_s, odom[:, 0], odom[:, 4]))) % 360.0
    return (status[:, 1] - true_heading + 180.0) % 360.0 - 180.0


def analyse_open_loop(odom, status, rc):
    t, yaw_rate = odom[:, 0], odom[:, 5]
    stick = (hold(t, rc[:, 0], rc[:, 1]) - 1500.0) / 500.0
    # the schedule's steps are >= 0.05; below that it is the heading-hold
    # loop (before the schedule) or zero: treat as "no step"
    stick = np.where(np.abs(stick) < 0.05, 0.0, stick)
    # step segments = constant stick runs
    edges = np.flatnonzero(np.abs(np.diff(stick)) > 1e-6) + 1
    bounds = np.r_[0, edges, len(t)]
    print('open-loop yaw-stick steps (yaw rate: body z, FLU, + = counter-clockwise):')
    print('   stick   dur    steady rate    per stick     rise10-90  overshoot'
          '  ripple(std)  horiz dist  alt dev   fw-heading err')
    rows = []
    baseline = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        s = stick[a]
        if b - a >= 50 and abs(s) < 1e-6 and t[a] > 1.0 and a > 0:
            pos0 = odom[a:b, 1:3]
            baseline.append(float(np.max(np.linalg.norm(pos0 - pos0[0], axis=1))))
        if b - a < 50 or abs(s) < 1e-6 or t[a] < 1.0:
            continue
        seg_t, seg_r = t[a:b] - t[a], yaw_rate[a:b]
        dur = seg_t[-1]
        steady = seg_r[seg_t > 0.5 * dur]
        r_ss = float(np.mean(steady))
        ripple = float(np.std(steady))
        r0 = float(yaw_rate[a - 1]) if a > 0 else 0.0
        span = r_ss - r0
        def first(frac):
            idx = np.flatnonzero((seg_r - r0) / span >= frac) if span != 0 else []
            return seg_t[idx[0]] if len(idx) else math.nan
        rise = first(0.9) - first(0.1)
        peak = float(np.max(seg_r * np.sign(r_ss)))  # magnitude in the turn direction
        overshoot = (peak - abs(r_ss)) / abs(r_ss) * 100.0 if r_ss != 0 else math.nan
        pos = odom[a:b, 1:4]
        horiz = float(np.max(np.linalg.norm(pos[:, :2] - pos[0, :2], axis=1)))
        alt = float(np.max(np.abs(pos[:, 2] - pos[0, 2])))
        in_seg = (status[:, 0] >= t[a]) & (status[:, 0] <= t[b - 1])
        fw_err = heading_error_deg(odom, status[in_seg]) if np.any(in_seg) else np.array([math.nan])
        per_stick = math.degrees(r_ss) / s
        rows.append((s, math.degrees(r_ss), per_stick, ripple, horiz, alt))
        print(f'  {s:+5.2f}  {dur:4.1f} s  {math.degrees(r_ss):+8.1f} deg/s  '
              f'{per_stick:+7.1f} deg/s  {rise:7.3f} s  {overshoot:7.1f} %  '
              f'{math.degrees(ripple):7.2f} deg/s  {horiz:7.3f} m  {alt:6.3f} m  '
              f'max {np.nanmax(np.abs(fw_err)):5.1f} deg')
    if baseline:
        print(f'  baseline (stick 0 gaps between steps): horizontal drift max '
              f'{max(baseline):.3f} m, mean {np.mean(baseline):.3f} m')
    if not rows:
        check(False, 'no stick steps recorded')
        return
    rows = np.array(rows)
    check(all(np.sign(r[1]) == -np.sign(r[0]) for r in rows),
          '+ stick turns clockwise (negative ENU yaw rate) for every step')
    small = rows[np.abs(rows[:, 0]) <= 0.4]
    spread = (np.max(np.abs(small[:, 2])) - np.min(np.abs(small[:, 2]))) / np.mean(np.abs(small[:, 2]))
    check(spread < 0.5, f'rate per stick consistent for |stick| <= 0.4 '
          f'({np.min(np.abs(small[:, 2])):.0f}..{np.max(np.abs(small[:, 2])):.0f} deg/s per unit, '
          f'spread {spread * 100:.0f} %)')
    worst_ripple = math.degrees(np.max(rows[:, 3]))
    check(worst_ripple < 10.0, f'no yaw oscillation/limit cycle (worst steady ripple '
          f'{worst_ripple:.2f} deg/s < 10)')
    check(np.max(rows[np.abs(rows[:, 0]) <= 0.4, 4]) < 0.5,
          f'position held while yawing (max {np.max(rows[np.abs(rows[:, 0]) <= 0.4, 4]):.3f} m < 0.5 m, |stick| <= 0.4)')
    check(np.max(rows[:, 5]) < 0.2, f'altitude held while yawing (max {np.max(rows[:, 5]):.3f} m < 0.2 m)')
    print(f'  -> yaw rate per unit stick (|stick| <= 0.4): '
          f'{np.mean(np.abs(small[:, 2])):.0f} deg/s')


def analyse_closed_loop(odom, status, sp):
    t = odom[:, 0]
    inside = (t >= sp[0, 0]) & (t <= sp[-1, 0])
    o = odom[inside]
    yaw_true = unwrap(odom[:, 4])[inside]
    yaw_sp = hold(o[:, 0], sp[:, 0], sp[:, 1])
    # align branches: the setpoint is unwrapped from the take-off heading
    offset = 2 * math.pi * round((yaw_sp[0] - yaw_true[0]) / (2 * math.pi))
    err = np.degrees(yaw_sp - (yaw_true + offset))
    moving = np.abs(np.gradient(yaw_sp, o[:, 0])) > 1e-3
    print(f'closed-loop heading: {len(o)} samples, setpoint range '
          f'{math.degrees(sp[:, 1].min() - sp[0, 1]):+.0f}..{math.degrees(sp[:, 1].max() - sp[0, 1]):+.0f} deg'
          f' from take-off heading')
    def st(v):
        return f'mean {np.mean(np.abs(v)):.2f}  rms {np.sqrt(np.mean(v ** 2)):.2f}  max {np.max(np.abs(v)):.2f} deg'
    print(f'  while turning (setpoint slewing)  |error| {st(err[moving])}')
    # settled: setpoint constant for > 2 s
    const_since = np.zeros(len(o))
    last = o[0, 0]
    for i in range(1, len(o)):
        if moving[i]:
            last = o[i, 0]
        const_since[i] = o[i, 0] - last
    settled = (~moving) & (const_since > 2.0)
    if np.any(settled):
        print(f'  settled (> 2 s after the turn)      |error| {st(err[settled])}')
    # per plateau
    change = np.flatnonzero(np.abs(np.diff(sp[:, 1])) > 1e-6)
    plateaus = []
    start = 0
    for i in list(change) + [len(sp) - 1]:
        if i - start > 5:
            plateaus.append((sp[start, 0], sp[i, 0], sp[i, 1]))
        start = i + 1
    print('  per heading plateau:')
    for t0, t1, value in plateaus:
        m = (o[:, 0] >= t0) & (o[:, 0] <= t1)
        if not np.any(m):
            continue
        e = err[m]
        print(f'    heading {math.degrees(value - sp[0, 1]):+7.1f} deg  from t={t0:6.2f} s '
              f'({t1 - t0:4.1f} s)  final error {e[-1]:+6.2f} deg  max |err| {np.max(np.abs(e)):5.2f} deg')
    # coupling window: the scan itself, from the first heading change (the
    # climb to the hover point comes before it), relative to the position
    # at that moment
    scan_start = sp[change[0], 0] if len(change) else sp[0, 0]
    scan = o[:, 0] >= scan_start
    horiz = np.linalg.norm(o[scan, 1:3] - o[scan][0, 1:3], axis=1)
    alt = np.abs(o[scan, 3] - o[scan][0, 3])
    print(f'  scan starts at t={scan_start:.2f} s')
    fw = heading_error_deg(odom, status[(status[:, 0] >= sp[0, 0]) & (status[:, 0] <= sp[-1, 0])])
    print(f'  position drift from the hover point during the scan: max {horiz.max():.3f} m, '
          f'mean {horiz.mean():.3f} m; altitude deviation max {alt.max():.3f} m')
    print(f'  firmware heading estimate - truth: mean {np.mean(fw):+.2f}  max |.| {np.max(np.abs(fw)):.2f} deg')
    total = abs(math.degrees(sp[:, 1].max() - sp[:, 1].min()))
    check(total >= 359.0, f'setpoint covered a full turn ({total:.0f} deg)')
    if np.any(settled):
        check(np.mean(np.abs(err[settled])) < 2.0,
              f'settled mean heading error {np.mean(np.abs(err[settled])):.2f} deg < 2 deg')
    check(np.max(np.abs(err[moving])) < 15.0 if np.any(moving) else True,
          f'tracking error while turning {np.max(np.abs(err[moving])) if np.any(moving) else 0:.1f} deg < 15 deg')
    check(horiz.max() < 0.5, f'position held during the scan ({horiz.max():.3f} m < 0.5 m)')
    check(alt.max() < 0.2, f'altitude held during the scan ({alt.max():.3f} m < 0.2 m)')
    check(np.max(np.abs(fw)) < 10.0,
          f'firmware heading estimate within 10 deg of truth (max {np.max(np.abs(fw)):.1f} deg)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['open_loop', 'closed_loop'])
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    rclpy.init()
    node = Recorder()
    clock = node.get_clock()
    start = clock.now()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (clock.now() - start).nanoseconds * 1e-9 > args.timeout:
            print('timeout')
            break
    odom, status = np.array(node.odom), np.array(node.status)
    rc = np.array(node.rc) if node.rc else np.zeros((0, 2))
    sp = np.array(node.sp) if node.sp else np.zeros((0, 2))
    node.destroy_node()
    rclpy.shutdown()
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        np.savetxt(os.path.join(args.out, 'odom.csv'), odom, delimiter=',', comments='',
                   header='t_s,x_enu_m,y_enu_m,z_enu_m,yaw_enu_rad,yaw_rate_flu_rad_s,tilt_rad')
        np.savetxt(os.path.join(args.out, 'fc_status.csv'), status, delimiter=',', comments='',
                   header='t_s,heading_deg_fw,armed')
        np.savetxt(os.path.join(args.out, 'rc_yaw.csv'), rc, delimiter=',', comments='',
                   header='t_s,yaw_us')
        np.savetxt(os.path.join(args.out, 'yaw_setpoint.csv'), sp, delimiter=',', comments='',
                   header='t_s,yaw_sp_rad')
    check(node.armed_seen and node.done, 'flew and disarmed')
    if len(odom) == 0 or len(status) == 0:
        print('RESULT: FAIL (no data)')
        return 1
    if args.mode == 'open_loop':
        analyse_open_loop(odom, status, rc)
    elif len(sp) == 0:
        check(False, 'no yaw setpoints received')
    else:
        analyse_closed_loop(odom, status, sp)
    print('RESULT: ' + ('PASS' if FAILURES == 0 else f'FAIL ({FAILURES} check(s))'))
    return 0 if FAILURES == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
