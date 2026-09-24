#!/usr/bin/env python3
"""Checks of the vehicle dynamics and coordinate frames in Gazebo.

Run against a PAUSED simulation started with the config generated for the
scenario (scripts/check_dynamics.sh does all of this). The world is unpaused
once the subscriptions are ready. Data: /sim/pluto/odometry (ground truth,
ENU world / FLU body) and /pluto/imu (gz IMU sensor on base_link).

Scenarios (vehicle spawned facing +x, i.e. ENU yaw 0, unless stated):
  rest      motors off on the ground: IMU specific force = (0, 0, +g) FLU,
            zero angular rate.
  freefall  motors off from 5 m: vertical motion matches the analytic
            solution with quadratic body drag,
            v(t) = v_t tanh(g t / v_t),  v_t = sqrt(m g / (1/2 rho CdA_z)).
  forward   position hold to a target 1 m ahead (+x): nose must go DOWN
            (FRD pitch < 0) and the vehicle must move +x, not sideways.
  right     position hold to a target 1 m to the right (-y): right side must
            go DOWN (FRD roll > 0) and the vehicle must move -y.
  yaw       heading setpoint north while facing east: the vehicle must turn
            counter-clockwise seen from above (FLU yaw rate > 0) by 90 deg
            and settle.
  In forward/right/yaw the IMU gyro (FLU) must agree with the rate of change
  of the ground-truth attitude.
"""

import argparse
import math
import subprocess
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu


def euler_zyx_from_quaternion(w, x, y, z):
    """ENU/FLU Z-Y-X Euler angles (roll, pitch, yaw) of the body."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Recorder(Node):
    def __init__(self):
        super().__init__('dynamics_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = []  # (t, x, y, z, roll_flu, pitch_flu, yaw_flu)
        self.imu = []   # (t, ax, ay, az, wx, wy, wz)
        self.create_subscription(Odometry, '/sim/pluto/odometry',
                                 self._on_odom, 200)
        self.create_subscription(Imu, '/pluto/imu', self._on_imu, 200)

    @staticmethod
    def _t(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom.append((self._t(msg.header.stamp), p.x, p.y, p.z,
                          *euler_zyx_from_quaternion(q.w, q.x, q.y, q.z)))

    def _on_imu(self, msg):
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.imu.append((self._t(msg.header.stamp), a.x, a.y, a.z,
                         w.x, w.y, w.z))

    def ready(self):
        return (self.count_publishers('/sim/pluto/odometry') > 0 and
                self.count_publishers('/pluto/imu') > 0)


def at(rows, t):
    return next((r for r in rows if r[0] >= t), rows[-1])


def window(rows, t0, t1):
    return [r for r in rows if t0 <= r[0] <= t1]


class Checks:
    def __init__(self):
        self.failures = 0

    def expect(self, ok, text):
        print(f'  [{"PASS" if ok else "FAIL"}] {text}')
        self.failures += 0 if ok else 1


def check_rest(rec, cfg, checks):
    g = cfg['vehicle']['gravity_m_s2']
    late = window(rec.imu, 0.5, 1e9)
    mean = [sum(r[i] for r in late) / len(late) for i in range(1, 7)]
    checks.expect(abs(mean[0]) < 0.01 and abs(mean[1]) < 0.01 and
                  abs(mean[2] - g) < 0.01,
                  f'IMU specific force FLU ({mean[0]:.3f}, {mean[1]:.3f}, '
                  f'{mean[2]:.3f}) = (0, 0, +{g})')
    checks.expect(max(abs(v) for v in mean[3:]) < 1e-3,
                  f'IMU angular rate ~0 ({mean[3]:.1e}, {mean[4]:.1e}, '
                  f'{mean[5]:.1e})')


def check_freefall(rec, cfg, checks):
    v = cfg['vehicle']
    g, m = v['gravity_m_s2'], v['mass_kg']
    k = 0.5 * v['drag']['air_density_kg_m3'] * \
        v['drag']['body_drag_area_frd_m2']['z'] / m
    v_t = math.sqrt(g / k)
    rows = [r for r in rec.odom if r[3] > 0.2]   # before ground contact
    # Released at rest at sim time 0 from the spawn height. (Starting the
    # clock at the first recorded sample would ignore the ~0.2 m/s already
    # gained by then.)
    t0, z0 = 0.0, cfg['gazebo_model']['spawn_height_m']
    worst = 0.0
    print('    t [s]   z gazebo   z analytic   z no-drag')
    for r in rows[::10]:
        t = r[0] - t0
        z_analytic = z0 - (v_t * v_t / g) * math.log(math.cosh(g * t / v_t))
        z_vacuum = z0 - 0.5 * g * t * t
        worst = max(worst, abs(r[3] - z_analytic))
        print(f'    {t:5.2f}   {r[3]:8.4f}   {z_analytic:10.4f}   '
              f'{z_vacuum:9.4f}')
    drop = z0 - rows[-1][3]
    checks.expect(worst < 0.01,
                  f'free fall matches analytic solution: worst error '
                  f'{worst * 1000:.1f} mm over a {drop:.2f} m drop (gravity '
                  f'and drag consistent to ~{100 * worst / drop:.2f} %), '
                  f'terminal velocity {v_t:.1f} m/s')


def gyro_agreement(rec, t0, t1):
    """Max |IMU gyro - d(attitude)/dt| (FLU, small-angle) in [t0, t1]."""
    odom = window(rec.odom, t0, t1)
    worst = 0.0
    for a, b in zip(odom, odom[1:]):
        dt = b[0] - a[0]
        if dt <= 0:
            continue
        rates = [(b[4] - a[4]) / dt, (b[5] - a[5]) / dt,
                 math.remainder(b[6] - a[6], 2 * math.pi) / dt]
        imu = at(rec.imu, 0.5 * (a[0] + b[0]))
        worst = max(worst, max(abs(rates[i] - imu[4 + i]) for i in range(3)))
    return worst


def check_translation(rec, checks, axis_name, move_axis, tilt_index,
                      tilt_sign_frd, flu_to_frd):
    early = window(rec.odom, 0.0, 1.5)
    tilt = [flu_to_frd * r[tilt_index] for r in early]
    extreme = min(tilt) if tilt_sign_frd < 0 else max(tilt)
    checks.expect(extreme * tilt_sign_frd > 0.02,
                  f'{axis_name}: FRD {"pitch" if tilt_index == 5 else "roll"}'
                  f' reaches {extreme:+.3f} rad (expected sign '
                  f'{"-" if tilt_sign_frd < 0 else "+"})')
    start, end = at(rec.odom, 0.0), at(rec.odom, 3.0)
    moved = [end[i] - start[i] for i in (1, 2)]
    want = moved[move_axis[0]] * move_axis[1]
    other = abs(moved[1 - move_axis[0]])
    checks.expect(want > 0.3 and other < 0.2 * want,
                  f'{axis_name}: moved dx={moved[0]:+.3f} dy={moved[1]:+.3f} m')
    worst = gyro_agreement(rec, 0.1, 3.0)
    checks.expect(worst < 0.05,
                  f'IMU gyro matches attitude rates (worst {worst:.3f} rad/s)')


def check_yaw(rec, checks):
    turning = window(rec.imu, 0.2, 1.0)
    mean_rate = sum(r[6] for r in turning) / len(turning)
    checks.expect(mean_rate > 0.1,
                  f'turns counter-clockwise from above (IMU yaw rate '
                  f'{mean_rate:+.3f} rad/s FLU)')
    final = at(rec.odom, 1e9)
    checks.expect(abs(final[6] - math.pi / 2) < 0.02,
                  f'settles facing north (ENU yaw {final[6]:.4f} rad, '
                  f'expected {math.pi / 2:.4f})')
    worst = gyro_agreement(rec, 0.1, 3.0)
    checks.expect(worst < 0.05,
                  f'IMU gyro matches attitude rates (worst {worst:.3f} rad/s)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('scenario',
                        choices=['rest', 'freefall', 'forward', 'right', 'yaw'])
    parser.add_argument('config')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--world', default='pluto_legacy')
    args = parser.parse_args()
    cfg = yaml.safe_load(open(args.config, encoding='utf-8'))

    rclpy.init()
    rec = Recorder()
    while rclpy.ok() and not rec.ready():
        rclpy.spin_once(rec, timeout_sec=0.1)
    subprocess.run(
        ['gz', 'service', '-s', f'/world/{args.world}/control', '--reqtype',
         'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', '--timeout',
         '5000', '--req', 'pause: false'], check=True, capture_output=True)
    while rclpy.ok() and (not rec.odom or rec.odom[-1][0] < args.duration):
        rclpy.spin_once(rec, timeout_sec=0.1)
    rec.destroy_node()
    rclpy.shutdown()
    # The OdometryPublisher's first sample differentiates from an undefined
    # previous pose; drop it.
    rec.odom = rec.odom[1:]

    print(f'{args.scenario}: {len(rec.odom)} odometry, {len(rec.imu)} IMU '
          'samples')
    checks = Checks()
    if args.scenario == 'rest':
        check_rest(rec, cfg, checks)
    elif args.scenario == 'freefall':
        check_freefall(rec, cfg, checks)
    elif args.scenario == 'forward':
        # FLU pitch = -FRD pitch; move along +x (index 0, sign +).
        check_translation(rec, checks, 'target ahead', (0, +1), 5, -1, -1)
    elif args.scenario == 'right':
        # FLU roll = FRD roll; move along -y (index 1, sign -).
        check_translation(rec, checks, 'target right', (1, -1), 4, +1, +1)
    else:
        check_yaw(rec, checks)
    print('RESULT:', 'PASS' if checks.failures == 0 else
          f'FAIL ({checks.failures})')
    return 0 if checks.failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
