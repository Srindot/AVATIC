#!/usr/bin/env python3
"""run_recorder: saves everything about one arena run to a directory.

Started by competition.launch.py (record:=true, the default). Files, all
in <run_dir>/ (plain CSV / YAML / MP4, read by analysis/runlog.py):

  meta.yaml        seed, controller, time limit, start time, files, status,
                   integrity (the fair-play check: integrity_monitor.py)
  layout.yaml      the arena used (balloon positions, colours, points)
  trajectory.csv   GROUND TRUTH (judges' view; the controller never sees it):
                   t_s, x/y/z_enu_m, vx/vy/vz_enu_m_s (world), roll/pitch/yaw
                   _rad (REP-103: yaw CCW from east), 100 Hz
  telemetry.csv    what the controller saw from the flight controller:
                   t_s, armed, ready_to_arm, roll_deg, pitch_deg (+ nose up),
                   heading_deg, altitude_m (baro), battery_v, altitude_hold
  commands.csv     what reached the drone (the RC stream, 50 Hz): t_s, the 8
                   channels in us and the normalised roll, pitch, yaw_rate
                   (+ = clockwise), throttle, altitude_hold, armed switch
  events.csv       t_s, kind (start | pop | end), colour, points, total, text
  result.yaml      the arena's final result (score, per-balloon popped);
                   written last, so it exists only once the run is saved
  result_arena.yaml  the same result, written by the arena itself
  camera.mp4       the camera, H.264, scaled to camera_width_px (default 640)
  camera_frames.csv  frame index -> t_s of each video frame

All times are simulation time. The run is complete when the arena result
arrives; on an early stop (Ctrl-C) the files are closed and meta.yaml says
status: incomplete.
"""

import csv
import math
import os
import shutil
import subprocess
from datetime import datetime

import rclpy
import rclpy.executors
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pluto_x_interfaces.msg import FlightControllerStatus, RcCommand


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def euler_zyx(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def rotate(q, v):
    """Body -> world rotation of vector v by quaternion q."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    return ((1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
            2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
            2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz)


class RunRecorder(Node):

    def __init__(self):
        super().__init__('run_recorder')
        self.run_dir = self.declare_parameter('run_dir', '').value
        if not self.run_dir:
            raise RuntimeError('run_recorder needs the run_dir parameter')
        self.meta = {
            'run_id': os.path.basename(self.run_dir.rstrip('/')),
            'started_local_time': datetime.now().isoformat(timespec='seconds'),
            'seed': self.declare_parameter('seed', '').value,
            'controller': self.declare_parameter('controller', '').value,
            'arena_config': self.declare_parameter('arena_config', '').value,
            'vehicle_config': self.declare_parameter('vehicle_config', '').value,
            'status': 'recording',
        }
        self.camera_width = int(self.declare_parameter('camera_width_px', 640).value)
        os.makedirs(self.run_dir, exist_ok=True)
        layout = os.path.join(self.run_dir, 'layout.yaml')
        if self.meta['arena_config'] and os.path.isfile(self.meta['arena_config']):
            if os.path.abspath(self.meta['arena_config']) != os.path.abspath(layout):
                shutil.copy(self.meta['arena_config'], layout)
            with open(self.meta['arena_config'], encoding='utf-8') as stream:
                arena = yaml.safe_load(stream)
            self.meta['time_limit_s'] = arena.get('time_limit_s')
            self.meta['clock_start'] = arena.get('clock_start')
        self.files, self.writers = {}, {}
        self._open('trajectory', ['t_s', 'x_enu_m', 'y_enu_m', 'z_enu_m', 'vx_enu_m_s',
                                  'vy_enu_m_s', 'vz_enu_m_s', 'roll_rad', 'pitch_rad', 'yaw_rad'])
        self._open('telemetry', ['t_s', 'armed', 'ready_to_arm', 'roll_deg', 'pitch_deg',
                                 'heading_deg', 'altitude_m', 'battery_v', 'altitude_hold'])
        self._open('commands', ['t_s', 'roll_us', 'pitch_us', 'throttle_us', 'yaw_us', 'aux1_us',
                                'aux2_us', 'aux3_us', 'aux4_us', 'roll', 'pitch', 'yaw_rate',
                                'throttle', 'altitude_hold', 'arm_switch'])
        self._open('events', ['t_s', 'kind', 'colour', 'points', 'total', 'text'])
        self._open('camera_frames', ['frame', 't_s'])
        self.video = None
        self.video_frames = 0
        self.finished = False
        self._write_meta()
        qos = QoSProfile(depth=50)
        self.create_subscription(Odometry, '/sim/pluto/odometry', self.on_odom, qos)
        self.create_subscription(FlightControllerStatus, '/pluto/fc_status', self.on_status, qos)
        self.create_subscription(RcCommand, '/pluto/rc', self.on_rc, qos)
        self.create_subscription(String, '/arena/events', self.on_event, qos)
        self.create_subscription(String, '/arena/result', self.on_result, qos)
        self.create_subscription(Image, '/pluto/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.integrity = None
        self.create_subscription(String, '/arena/integrity', self.on_integrity, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(f'recording this run to {self.run_dir}')

    def _open(self, name, header):
        f = open(os.path.join(self.run_dir, f'{name}.csv'), 'w', newline='', encoding='utf-8')
        w = csv.writer(f)
        w.writerow(header)
        self.files[name], self.writers[name] = f, w

    def _write_meta(self):
        with open(os.path.join(self.run_dir, 'meta.yaml'), 'w', encoding='utf-8') as out:
            yaml.safe_dump(self.meta, out, sort_keys=False)

    # ---------------------------------------------------------------- data
    def on_odom(self, m):
        if self.finished:
            return
        q, p = m.pose.pose.orientation, m.pose.pose.position
        t = m.twist.twist.linear
        v = rotate(q, (t.x, t.y, t.z))  # odometry twist is in the body frame
        roll, pitch, yaw = euler_zyx(q)
        self.writers['trajectory'].writerow(
            [f'{stamp_s(m.header.stamp):.3f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}',
             f'{v[0]:.4f}', f'{v[1]:.4f}', f'{v[2]:.4f}', f'{roll:.5f}', f'{pitch:.5f}',
             f'{yaw:.5f}'])

    def on_status(self, m):
        if self.finished:
            return
        self.writers['telemetry'].writerow(
            [f'{stamp_s(m.header.stamp):.3f}', int(m.armed), int(m.ok_to_arm),
             f'{m.roll_deg:.2f}', f'{m.pitch_deg:.2f}', f'{m.heading_deg:.1f}',
             f'{m.altitude_m:.3f}', f'{m.battery_v:.3f}', int(m.altitude_hold)])

    def on_rc(self, m):
        if self.finished:
            return
        ch = [m.roll_us, m.pitch_us, m.throttle_us, m.yaw_us, m.aux1_us, m.aux2_us,
              m.aux3_us, m.aux4_us]
        self.writers['commands'].writerow(
            [f'{stamp_s(m.header.stamp):.3f}', *ch, f'{(ch[0] - 1500) / 500:.3f}',
             f'{(ch[1] - 1500) / 500:.3f}', f'{(ch[3] - 1500) / 500:.3f}',
             f'{(ch[2] - 1000) / 1000:.3f}', int(1300 <= ch[6] <= 2100),
             int(1300 <= ch[7] <= 2100)])

    def on_event(self, m):
        if self.finished:
            return
        text = m.data
        try:
            t = float(text.split('t=')[1].split(' s')[0])
        except (IndexError, ValueError):
            t = float('nan')
        kind, colour, points, total = 'other', '', '', ''
        if ' POP ' in text:
            kind = 'pop'
            try:
                after = text.split(' POP ')[1]
                colour = after.split()[0]
                points = after.split()[1].lstrip('+')
                total = text.rsplit('total ', 1)[1].strip()
            except IndexError:
                pass
        elif 'run clock started' in text:
            kind = 'start'
        elif 'TIME UP' in text:
            kind = 'end'
        self.writers['events'].writerow([f'{t:.3f}', kind, colour, points, total, text])
        self.files['events'].flush()

    def on_integrity(self, m):
        self.integrity = yaml.safe_load(m.data)

    def on_result(self, m):
        if self.finished:
            return
        result = yaml.safe_load(m.data)
        self.meta.update(status='complete', score=result.get('score'),
                         max_score=result.get('max_score'), popped=result.get('popped'))
        self.finish()
        # written last: once result.yaml exists, every other file is complete
        path = os.path.join(self.run_dir, 'result.yaml')
        with open(path + '.tmp', 'w', encoding='utf-8') as out:
            out.write(m.data)
        os.replace(path + '.tmp', path)
        self.get_logger().info(f"run saved: {self.run_dir} (score {result.get('score')})")

    def on_image(self, m):
        if self.finished or m.encoding != 'rgb8' or self.video is False:
            return
        if self.video is None:
            height = round(m.height * self.camera_width / m.width / 2) * 2
            try:
                self.video = self._start_video(m, height)
            except FileNotFoundError:
                self.video = False  # the CSV recording goes on without camera.mp4
                self.get_logger().error('ffmpeg not found (sudo apt install ffmpeg): '
                                        'no camera.mp4 for this run')
                return
        data = bytes(m.data)
        row = m.width * 3
        if m.step != row:   # rows padded: ffmpeg needs them packed
            data = b''.join(data[r * m.step:r * m.step + row] for r in range(m.height))
        try:
            self.video.stdin.write(data)
        except (BrokenPipeError, OSError):
            return
        self.writers['camera_frames'].writerow([self.video_frames, f'{stamp_s(m.header.stamp):.3f}'])
        self.video_frames += 1

    def _start_video(self, m, height):
        # -r 18 is only the nominal container rate: camera_frames.csv has the
        # true time of every frame (the rendered rate is ~17.3 Hz)
        return subprocess.Popen(
                ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                 '-pix_fmt', 'rgb24', '-s', f'{m.width}x{m.height}', '-r', '18', '-i', 'pipe:0',
                 '-vf', f'scale={self.camera_width}:{height}', '-c:v', 'libx264',
                 '-preset', 'veryfast', '-crf', '28', '-pix_fmt', 'yuv420p',
                 os.path.join(self.run_dir, 'camera.mp4')],
                stdin=subprocess.PIPE)

    # -------------------------------------------------------------- finish
    def finish(self):
        if self.finished:
            return
        self.finished = True
        if self.meta['status'] == 'recording':
            self.meta['status'] = 'incomplete'
        for f in self.files.values():
            f.close()
        if self.video:
            try:
                self.video.stdin.close()
                self.video.wait(timeout=20)
            except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                self.video.kill()   # a second Ctrl-C: still write meta.yaml below
        self.meta['camera_frames'] = self.video_frames
        self.meta['integrity'] = self.integrity or {'verdict': 'not checked'}
        self.meta['finished_local_time'] = datetime.now().isoformat(timespec='seconds')
        self._write_meta()


def main():
    rclpy.init()
    node = RunRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()  # files first: a second Ctrl-C below must not lose data
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
