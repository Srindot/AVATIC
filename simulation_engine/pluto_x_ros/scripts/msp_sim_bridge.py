#!/usr/bin/env python3
"""msp_sim_bridge: the simulated Pluto X behind a real MSP-over-TCP link.

Test bridge for the hardware backend (hitl/avatic_hitl/hardware.py) before the drone is available: the client speaks genuine MSP
v1 bytes over TCP, exactly as to the camera module (192.168.0.1:9060), and
this node turns them into the simulator's RC topic and answers telemetry
requests from the simulated flight controller.

  MSP (TCP, default 127.0.0.1:9060)
    MSP_SET_RAW_RC 200  -> /pluto/rc (pluto_x_interfaces/RcCommand)
    MSP_SET_COMMAND 217 -> 2 = land: emulated (the simulator's firmware gets
                           RC only), as the firmware lands (command.cpp,
                           mw.cpp:379): it forces the throttle channel to
                           1300 us, which is a 50 cm/s descent in altitude
                           hold (AUX3 on) but a raw 30 % throttle - a drop -
                           without it; it disarms once no longer
                           descending (after 2 s) or after 30 s. A client
                           disarm (AUX4 off) ends it at once. Other
                           commands are acknowledged
    MSP_STATUS / ATTITUDE / ALTITUDE / ANALOG / BOXIDS -> replies built from
                           /pluto/fc_status in the MagisV2 layouts
  video (TCP, default 127.0.0.1:9061)
    raw H.264 (Annex B) of /pluto/camera/image_raw, encoded by ffmpeg
    (libx264, zerolatency, a keyframe every second); a client reads it like
    the camera module's stream

The MSP encoding here is written independently of the client's (hitl/avatic_hitl/msp.py)
so the test checks one implementation against the other; both follow
firmware/magisv2/src/main/io/serial_msp.cpp.
"""

import socket
import struct
import subprocess
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image

from pluto_x_interfaces.msg import FlightControllerStatus, RcCommand

MSP_STATUS, MSP_ATTITUDE, MSP_ALTITUDE, MSP_ANALOG, MSP_BOXIDS = 101, 108, 109, 110, 119
MSP_SET_RAW_RC, MSP_SET_COMMAND = 200, 217
BOX_IDS = bytes([0, 1, 3])  # ARM, ANGLE, BARO (the Pluto receiver setup)
LAND_DESCENT_M_S = 0.5        # emulated landing in altitude hold: the firmware's 50 cm/s,
LAND_GAIN_PER_M_S = 0.25      # ... throttle = hover + gain * (target - climb rate)
LAND_HOVER = 0.76             # the simulated Pluto X's hover throttle
LAND_RAW_THROTTLE = 0.30      # without altitude hold: the firmware's raw 1300 us
LAND_DISARM_AFTER_S = 2.0     # firmware: disarm when not descending after 2 s ...
LAND_TIMEOUT_S = 30.0         # ... or after 30 s


def frame(direction: bytes, cmd: int, payload: bytes = b'') -> bytes:
    c = len(payload) ^ cmd
    for b in payload:
        c ^= b
    return b'$M' + direction + bytes([len(payload), cmd]) + payload + bytes([c & 0xFF])


class MspSimBridge(Node):

    def __init__(self):
        super().__init__('msp_sim_bridge')
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.msp_port = int(self.declare_parameter('msp_port', 9060).value)
        self.video_port = int(self.declare_parameter('video_port', 9061).value)
        self.lock = threading.Lock()
        self.status = None
        self.client_rc = None          # last RC from the client (8 channels)
        self.land_started = None       # emulated landing in progress: start (sim time)
        self.hold_disarmed = False     # after an emulated disarm: until the client disarms too
        self.land_last_alt, self.land_climb_rate = None, 0.0
        self.land_warned_raw = False
        self.rc_pub = self.create_publisher(RcCommand, '/pluto/rc', QoSProfile(depth=10))
        self.create_subscription(FlightControllerStatus, '/pluto/fc_status', self.on_status,
                                 QoSProfile(depth=10))
        self.create_timer(0.02, self.landing_tick)
        threading.Thread(target=self.msp_server, daemon=True).start()
        self.video_clients = []
        self.encoder = None
        if self.video_port:
            self.create_subscription(Image, '/pluto/camera/image_raw', self.on_image,
                                     qos_profile_sensor_data)
            threading.Thread(target=self.video_server, daemon=True).start()
        self.get_logger().info(
            f'MSP on tcp://{self.host}:{self.msp_port}'
            + (f', H.264 video on tcp://{self.host}:{self.video_port}' if self.video_port else ''))

    # ---------------------------------------------------------------- MSP
    def on_status(self, msg):
        with self.lock:
            self.status = msg

    def now_s(self):
        """Simulation time (the launch sets use_sim_time)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_rc(self, channels):
        msg = RcCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        (msg.roll_us, msg.pitch_us, msg.throttle_us, msg.yaw_us, msg.aux1_us, msg.aux2_us,
         msg.aux3_us, msg.aux4_us) = channels
        self.rc_pub.publish(msg)

    def landing_tick(self):
        """Emulated firmware landing (see the module docstring)."""
        with self.lock:
            if self.land_started is None or self.client_rc is None:
                return
            s = self.status
            channels = list(self.client_rc)
            elapsed = self.now_s() - self.land_started
            if s is not None:
                if self.land_last_alt is not None:
                    rate = (s.altitude_m - self.land_last_alt) / 0.02
                    self.land_climb_rate += 0.2 * (rate - self.land_climb_rate)
                self.land_last_alt = s.altitude_m
            altitude_hold = 1300 <= channels[6] <= 2100
            if altitude_hold:
                throttle = min(max(LAND_HOVER + LAND_GAIN_PER_M_S *
                                   (-LAND_DESCENT_M_S - self.land_climb_rate), 0.3), 0.9)
            else:
                throttle = LAND_RAW_THROTTLE
                if not self.land_warned_raw:
                    self.land_warned_raw = True
                    self.get_logger().warn('landing WITHOUT altitude hold: the firmware uses a '
                                           'raw 1300 us throttle - the drone drops')
            disarm = elapsed > LAND_TIMEOUT_S or (
                elapsed > LAND_DISARM_AFTER_S and self.land_climb_rate > -0.08)
            if disarm or (s is not None and not s.armed and elapsed > 0.5):
                self.land_started = None
                self.hold_disarmed = True   # the firmware would not re-arm by itself
                channels[7] = 1000                              # firmware mwDisarm()
        channels[0] = channels[1] = channels[3] = 1500          # level, no yaw
        channels[2] = round(1000 + 1000 * throttle)
        # the descent above emulates the firmware's altitude-hold landing; the
        # simulated firmware itself flies it in angle mode (its altitude hold
        # assumes hover at 1500 us: docs/architecture.md firmware finding 7)
        channels[6] = 1000
        self.publish_rc(channels)

    def reply(self, cmd):
        with self.lock:
            s = self.status
        if cmd == MSP_BOXIDS:
            return frame(b'>', cmd, BOX_IDS)
        if s is None:
            return frame(b'!', cmd)
        if cmd == MSP_STATUS:
            flags = (s.armed << 8) | ((s.ok_to_arm and not s.armed) << 7) | \
                    ((not s.ok_to_arm and not s.armed) << 6)
            modes = s.armed | (s.angle_mode << 1) | (s.altitude_hold << 2)
            return frame(b'>', cmd, struct.pack('<HHHIB', flags, 0, 0b011, modes, 0))
        if cmd == MSP_ATTITUDE:
            return frame(b'>', cmd, struct.pack(
                '<hhh', round(s.roll_deg * 10), round(-s.pitch_deg * 10),  # firmware: nose down +
                round(s.heading_deg) % 360))
        if cmd == MSP_ALTITUDE:
            return frame(b'>', cmd, struct.pack('<ih', round(s.altitude_m * 100), 0))
        if cmd == MSP_ANALOG:
            return frame(b'>', cmd, struct.pack('<HHHHBB', round(s.battery_v * 1000),
                                                0, 0, 0, 0, 0))
        return frame(b'!', cmd)

    def handle_request(self, cmd, payload):
        if cmd == MSP_SET_RAW_RC:
            n = len(payload) // 2
            channels = list(struct.unpack(f'<{n}H', payload[:2 * n]))[:8]
            channels += [1000] * (8 - len(channels))
            with self.lock:
                self.client_rc = channels
                arm_on = 1300 <= channels[7] <= 2100
                if self.land_started is not None and not arm_on:
                    self.land_started = None     # client disarm (emergency stop) wins
                if not arm_on:
                    self.hold_disarmed = False   # the client toggled its switch: may re-arm
                elif self.hold_disarmed:
                    channels = channels[:7] + [1000]
                landing = self.land_started is not None
            if not landing:
                self.publish_rc(channels)
            return frame(b'>', cmd)
        if cmd == MSP_SET_COMMAND:
            if not payload:
                return frame(b'!', cmd)
            value = struct.unpack('<H', payload[:2])[0] if len(payload) >= 2 else payload[0]
            if value == 2:
                with self.lock:
                    armed = self.status is not None and self.status.armed
                    start = armed and self.land_started is None   # repeats are ignored
                    if start:
                        self.land_started = self.now_s()
                        self.land_last_alt, self.land_climb_rate = None, 0.0
                if start:
                    self.get_logger().info('MSP_SET_COMMAND land: emulated landing')
            return frame(b'>', cmd)
        return self.reply(cmd)

    def msp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.msp_port))
        server.listen(1)
        while rclpy.ok():
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info(f'MSP client {addr[0]}:{addr[1]} connected')
            buf = bytearray()
            try:
                while True:
                    data = conn.recv(4096)
                    if not data:
                        break
                    buf += data
                    while True:
                        i = buf.find(b'$M<')
                        if i < 0:
                            del buf[:max(len(buf) - 2, 0)]
                            break
                        del buf[:i]
                        if len(buf) < 6 or len(buf) < 6 + buf[3]:
                            break
                        size, cmd = buf[3], buf[4]
                        payload = bytes(buf[5:5 + size])
                        check = size ^ cmd
                        for b in payload:
                            check ^= b
                        ok = buf[5 + size] == check
                        del buf[:6 + size]
                        if ok:
                            try:
                                reply = self.handle_request(cmd, payload)
                            except Exception as error:  # noqa: BLE001 - one bad packet
                                self.get_logger().warn(f'MSP {cmd}: {error}')
                                reply = frame(b'!', cmd)
                            conn.sendall(reply)
            except OSError:
                pass
            self.get_logger().info('MSP client disconnected')
            conn.close()

    # -------------------------------------------------------------- video
    def on_image(self, msg):
        if self.encoder is False:
            return                      # no ffmpeg: MSP keeps working, no video
        if self.encoder is None:
            try:
                self.encoder = self._start_encoder(msg)
            except FileNotFoundError:
                self.encoder = False
                self.get_logger().error('ffmpeg not found (sudo apt install ffmpeg): no video')
                return
            threading.Thread(target=self.broadcast, daemon=True).start()
        if msg.encoding != 'rgb8' or msg.step != msg.width * 3:
            return
        try:
            self.encoder.stdin.write(bytes(msg.data))
            self.encoder.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def _start_encoder(self, msg):
        return subprocess.Popen(
                ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
                 '-pix_fmt', 'rgb24', '-s', f'{msg.width}x{msg.height}', '-r', '18',
                 '-i', 'pipe:0', '-c:v', 'libx264', '-preset', 'ultrafast',
                 '-tune', 'zerolatency', '-g', '18', '-x264-params', 'repeat-headers=1',
                 '-pix_fmt', 'yuv420p',
                 '-f', 'h264', 'pipe:1'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def broadcast(self):
        while True:
            chunk = self.encoder.stdout.read1(65536)
            if not chunk:
                return
            with self.lock:
                clients = list(self.video_clients)
            for c in clients:
                try:
                    c.sendall(chunk)
                except OSError:  # incl. the send timeout
                    with self.lock:
                        self.video_clients.remove(c)
                    c.close()

    def video_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.video_port))
        server.listen(4)
        while rclpy.ok():
            conn, addr = server.accept()
            conn.settimeout(1.0)  # a client that stops reading is dropped, not waited for
            self.get_logger().info(f'video client {addr[0]}:{addr[1]} connected')
            with self.lock:
                self.video_clients.append(conn)


def main():
    rclpy.init()
    node = MspSimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.encoder:
            node.encoder.kill()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
