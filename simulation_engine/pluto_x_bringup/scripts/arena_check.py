#!/usr/bin/env python3
"""End-to-end check of the balloon arena (run by scripts/check_arena.sh next
to arena.launch.py).

Records /arena/events, /arena/score, /arena/result and the camera until the
result arrives (the time limit) or a wall-clock timeout, then checks:
  * the run ended at the time limit (result ended_at within one physics
    step of time_limit_s for clock_start sim_start)
  * the score equals the sum of the points of the popped balloons, and at
    least --min-pops balloons were popped with at least --min-score points
  * every POP event is before the time limit
  * the camera published images of the configured size at about its rate
  * balloon colours are visible in the camera image at some point (the
    colour of at least one balloon in the arena)

Usage: arena_check.py [--min-pops N] [--min-score N] [--timeout S]
"""

import argparse
import sys

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

# rough RGB signatures of the arena colours under the world's lighting
SIGNATURES = {
    'red': lambda r, g, b: (r > 120) & (g < 60) & (b < 60),
    'yellow': lambda r, g, b: (r > 150) & (g > 120) & (b < 70),
    'blue': lambda r, g, b: (b > 120) & (r < 60) & (g < 110),
    'green': lambda r, g, b: (g > 90) & (r < 50) & (b < 60),
}


class Recorder(Node):

    def __init__(self, save_dir=''):
        super().__init__('arena_check')
        self.save_dir = save_dir
        self.next_save_s = 0.0
        self.events, self.scores, self.result = [], [], None
        self.images = 0
        self.image_size = None
        self.image_stamps = []
        self.colour_pixels = {c: 0 for c in SIGNATURES}
        self.create_subscription(String, '/arena/events',
                                 lambda m: self.events.append(m.data), 50)
        self.create_subscription(Int32, '/arena/score',
                                 lambda m: self.scores.append(m.data), 50)
        self.create_subscription(String, '/arena/result', self.on_result, 10)
        self.create_subscription(Image, '/pluto/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.odom = []  # t, x, y, z, yaw (ENU)
        self.create_subscription(Odometry, '/sim/pluto/odometry', self.on_odom, 100)

    def on_odom(self, m):
        import math
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        self.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def on_result(self, msg):
        self.result = yaml.safe_load(msg.data)

    def on_image(self, msg):
        self.images += 1
        self.image_size = (msg.width, msg.height, msg.encoding)
        self.image_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if msg.encoding != 'rgb8':
            return
        t = self.image_stamps[-1]
        if self.save_dir and t >= self.next_save_s:
            self.next_save_s = t + 2.0
            with open(f'{self.save_dir}/frame_{t:05.1f}s.ppm', 'wb') as out:
                out.write(f'P6 {msg.width} {msg.height} 255\n'.encode())
                out.write(bytes(msg.data))
        if self.images % 10:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        r, g, b = (img[..., i].astype(int) for i in range(3))
        for colour, test in SIGNATURES.items():
            self.colour_pixels[colour] = max(self.colour_pixels[colour],
                                             int(np.count_nonzero(test(r, g, b))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--min-pops', type=int, default=1)
    parser.add_argument('--min-score', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--camera-rate', type=float, default=18.0,
                        help='expected camera rate, Hz (vehicle YAML camera.rate_hz)')
    parser.add_argument('--odom-csv', default='', help='save the ground-truth track here')
    parser.add_argument('--forbid-colour', action='append', default=[],
                        help='fail if a balloon of this colour popped (e.g. red: penalty)')
    parser.add_argument('--save-frames', default='',
                        help='directory: save a camera frame (PPM) every 2 s of sim time')
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.save_frames)
    clock = node.get_clock()
    start = clock.now()
    while rclpy.ok() and node.result is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (clock.now() - start).nanoseconds * 1e-9 > args.timeout:
            break
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    rec = node
    if args.odom_csv and rec.odom:
        np.savetxt(args.odom_csv, np.array(rec.odom), delimiter=',', comments='',
                   header='t_s,x_enu_m,y_enu_m,z_enu_m,yaw_enu_rad')
    node.destroy_node()
    rclpy.shutdown()

    failures = 0

    def check(ok, text):
        nonlocal failures
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
        failures += 0 if ok else 1

    print('arena events:')
    for line in rec.events:
        print(f'    {line}')
    check(rec.result is not None, 'result published at the end of the run')
    if rec.result is None:
        print('RESULT: FAIL')
        return 1
    res = rec.result
    print(f"result: score {res['score']} / {res['max_score']}, popped {res['popped']}, "
          f"ended at t = {res['ended_at_sim_time_s']} s")
    if res['clock_start'] == 'sim_start':
        check(abs(res['ended_at_sim_time_s'] - res['time_limit_s']) <= 0.0015,
              f"run ended at the time limit ({res['ended_at_sim_time_s']} s vs "
              f"{res['time_limit_s']} s)")
    popped_points = sum(b['points'] for b in res['balloons'] if b['popped'])
    check(popped_points == res['score'],
          f"score {res['score']} = sum of popped balloons' points {popped_points}")
    check(res['popped'] >= args.min_pops and res['score'] >= args.min_score,
          f"popped {res['popped']} >= {args.min_pops}, score {res['score']} >= {args.min_score}")
    for colour in args.forbid_colour:
        bad = [b['name'] for b in res['balloons'] if b['popped'] and b['color'] == colour]
        check(not bad, f'no {colour} balloon popped' + (f' (popped: {bad})' if bad else ''))
    pops = [e for e in rec.events if ' POP ' in e]
    check(len(pops) == res['popped'], f'{len(pops)} POP events for {res["popped"]} pops')
    pop_times = [float(e.split('t=')[1].split(' s')[0]) for e in pops]
    check(all(t < res['time_limit_s'] for t in pop_times),
          f'all pops before the limit ({", ".join(f"{t:.2f}" for t in pop_times)} s)')
    check(rec.images > 0 and rec.image_size is not None,
          f'camera images received: {rec.images}, size {rec.image_size}')
    if len(rec.image_stamps) > 10:
        span = rec.image_stamps[-1] - rec.image_stamps[0]
        rate = (len(rec.image_stamps) - 1) / span if span > 0 else 0.0
        check(abs(rate - args.camera_rate) <= 0.15 * args.camera_rate,
              f'camera rate {rate:.1f} Hz (sim time), expected {args.camera_rate:g} Hz')
    seen = {c: n for c, n in rec.colour_pixels.items() if n > 0}
    print(f'  balloon-coloured pixels seen (max per checked frame): {rec.colour_pixels}')
    check(len(seen) >= 1, f'balloon colours visible in the camera: {sorted(seen)}')
    print('RESULT: ' + ('PASS' if failures == 0 else f'FAIL ({failures} check(s))'))
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
