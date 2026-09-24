"""The participant interface to the simulated Pluto X.

Plain Python: create a Drone, then read the camera and telemetry and send
stick commands. ROS is used underneath and never needs to be touched.

    from avatic_drone import Drone

    drone = Drone()                  # connects to the running simulator
    drone.wait_until_ready()         # flight controller calibrated
    drone.arm()                      # the arena's run clock starts now
    for _ in drone.loop(hz=20):      # until time is up (or Ctrl-C)
        frame = drone.get_frame()    # latest camera image (or None)
        tel = drone.get_telemetry()  # flight-controller estimates
        drone.send_command(roll=0.0, pitch=0.0, yaw_rate=0.0, throttle=0.76)
    drone.disarm()

What the drone understands (MagisV2 angle mode, as on the real Pluto X):

  roll      -1..1  bank angle setpoint;  + = bank right   (~32 deg at 1.0)
  pitch     -1..1  tilt angle setpoint;  + = nose down = move forward
  yaw_rate  -1..1  turn rate;            + = clockwise    (~77 deg/s at 1.0)
  throttle   0..1  collective thrust;    hover ~0.76 (depends on battery)

roll and pitch are ANGLES the firmware holds, not rates; only yaw is a
rate. The numbers in brackets were measured in the simulator.

What you get is only what the real drone provides: the camera image and
the flight controller's own estimates (attitude, heading, barometric
altitude, battery). There is no position or velocity.

All times are SIMULATION time (seconds).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Iterator, List, Optional

import numpy as np

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64, Int32, String

    from pluto_x_interfaces.msg import FlightControllerStatus, RcCommand
    from pluto_x_autonomy.api import StickCommand
    from pluto_x_autonomy.rc_mapping import to_rc
except ImportError as error:  # pragma: no cover - environment problem
    raise ImportError(
        'avatic_drone needs the simulator environment. Run first:\n'
        '    source /opt/ros/humble/setup.bash && source <repo>/install/setup.bash\n'
        f'({error})') from error

COMMAND_RATE_HZ = 50.0        # stick commands are streamed at the MSP rate
COMMAND_TIMEOUT_S = 0.5       # no send_command() for this long -> sticks neutral, throttle 0
ARM_TIMEOUT_S = 3.0


@dataclass(frozen=True)
class Frame:
    """One camera image. image: H x W x 3 uint8, RGB."""
    image: np.ndarray
    time_s: float
    width: int
    height: int


@dataclass(frozen=True)
class Telemetry:
    """The flight controller's own estimates (not ground truth).

    roll + = right side down, pitch + = nose up, heading clockwise from
    north (deg), altitude from the barometer relative to the start (m).
    """
    time_s: float
    armed: bool
    ready_to_arm: bool
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    altitude_m: float
    battery_v: float
    altitude_hold: bool


@dataclass
class ArenaStatus:
    score: int = 0
    time_remaining_s: Optional[float] = None
    finished: bool = False
    events: List[str] = field(default_factory=list)
    result: Optional[str] = None   # final summary (YAML text) once finished


class _Link(Node):
    """ROS side: subscriptions, the command stream. Runs in a thread."""

    def __init__(self):
        super().__init__('avatic_drone', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.image_msg = None
        self.status = None
        self.arena = ArenaStatus()
        self.command = StickCommand()
        self.arm_switch = False
        self.last_command_s = None
        self.timed_out = False
        qos = QoSProfile(depth=10)
        self.create_subscription(Image, '/pluto/camera/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(FlightControllerStatus, '/pluto/fc_status',
                                 self._on_status, qos)
        self.create_subscription(Int32, '/arena/score', self._on_score, qos)
        self.create_subscription(Float64, '/arena/time_remaining', self._on_remaining, qos)
        self.create_subscription(String, '/arena/events', self._on_event, 50)
        self.create_subscription(String, '/arena/result', self._on_result, qos)
        self.rc_pub = self.create_publisher(RcCommand, '/pluto/rc', qos)
        self.create_timer(1.0 / COMMAND_RATE_HZ, self._publish_command)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs
    def _on_image(self, msg):
        with self.lock:
            self.image_msg = msg

    def _on_status(self, msg):
        with self.lock:
            self.status = msg

    def _on_score(self, msg):
        with self.lock:
            self.arena.score = msg.data

    def _on_remaining(self, msg):
        with self.lock:
            self.arena.time_remaining_s = msg.data

    def _on_event(self, msg):
        with self.lock:
            self.arena.events.append(msg.data)

    def _on_result(self, msg):
        with self.lock:
            self.arena.result = msg.data
            self.arena.finished = True

    # -- output: the latest command, streamed at 50 Hz (sim time)
    def _publish_command(self):
        now = self.now_s()
        with self.lock:
            command, arm = self.command, self.arm_switch
            if (self.last_command_s is not None and
                    now - self.last_command_s > COMMAND_TIMEOUT_S):
                if not self.timed_out:
                    self.get_logger().warn(
                        f'no send_command() for {COMMAND_TIMEOUT_S} s: sticks neutral, throttle 0')
                    self.timed_out = True
                command = StickCommand()
        rc = to_rc(command, arm)
        msg = RcCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        (msg.roll_us, msg.pitch_us, msg.throttle_us, msg.yaw_us, msg.aux1_us,
         msg.aux2_us, msg.aux3_us, msg.aux4_us) = rc.as_list()
        self.rc_pub.publish(msg)


class Drone:
    """Connection to the simulated Pluto X (see the module docstring)."""

    def __init__(self, connect_timeout_s: float = 30.0):
        if not rclpy.ok():
            rclpy.init()
        self._link = _Link()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._link)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        deadline = time.monotonic() + connect_timeout_s
        while self._link.now_s() <= 0.0 or self._link.status is None:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    'no simulator found: is it running? '
                    '(ros2 launch pluto_x_bringup competition.launch.py)')
            time.sleep(0.05)

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races
            pass

    # ---------------------------------------------------------------- inputs

    def get_frame(self) -> Optional[Frame]:
        """Latest camera image (RGB, H x W x 3 uint8), or None before the first."""
        with self._link.lock:
            msg = self._link.image_msg
        if msg is None:
            return None
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return Frame(image=image.copy(), time_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     width=msg.width, height=msg.height)

    def get_telemetry(self) -> Optional[Telemetry]:
        """The flight controller's latest status, or None before the first."""
        with self._link.lock:
            s = self._link.status
        if s is None:
            return None
        return Telemetry(time_s=s.header.stamp.sec + s.header.stamp.nanosec * 1e-9,
                         armed=s.armed, ready_to_arm=s.ok_to_arm,
                         roll_deg=s.roll_deg, pitch_deg=s.pitch_deg,
                         heading_deg=s.heading_deg, altitude_m=s.altitude_m,
                         battery_v=s.battery_v, altitude_hold=s.altitude_hold)

    def arena(self) -> ArenaStatus:
        """Score, time left, events, and the result once the run is over."""
        with self._link.lock:
            a = self._link.arena
            return ArenaStatus(a.score, a.time_remaining_s, a.finished, list(a.events), a.result)

    def time(self) -> float:
        """Simulation time, seconds."""
        return self._link.now_s()

    # --------------------------------------------------------------- outputs

    def send_command(self, roll: float = 0.0, pitch: float = 0.0,
                     yaw_rate: float = 0.0, throttle: float = 0.0,
                     altitude_hold: bool = False) -> None:
        """Stick command, held until the next call (see the module docstring
        for the meaning and range of each value; out-of-range values are
        clipped). Call it at least every 0.5 s: otherwise the sticks go
        neutral with throttle 0 (as if the link was lost)."""
        for name, value in (('roll', roll), ('pitch', pitch), ('yaw_rate', yaw_rate),
                            ('throttle', throttle)):
            if not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number, got {value}')
        command = StickCommand(roll=roll, pitch=pitch, yaw=yaw_rate, throttle=throttle,
                               altitude_hold=altitude_hold).clipped()
        with self._link.lock:
            self._link.command = command
            self._link.last_command_s = self._link.now_s()
            self._link.timed_out = False

    # --------------------------------------------------------------- arming

    def wait_until_ready(self, timeout_s: float = 60.0) -> None:
        """Waits until the flight controller has calibrated and can arm."""
        deadline = time.monotonic() + timeout_s
        while True:
            t = self.get_telemetry()
            if t is not None and t.ready_to_arm:
                return
            if time.monotonic() > deadline:
                raise TimeoutError('flight controller not ready to arm')
            time.sleep(0.05)

    def arm(self) -> None:
        """Arms the motors (throttle at minimum). In the competition arena
        the run clock starts when the drone arms."""
        self.send_command(throttle=0.0)
        with self._link.lock:
            self._link.arm_switch = True
        deadline = time.monotonic() + ARM_TIMEOUT_S * 3
        while not (self.get_telemetry() and self.get_telemetry().armed):
            self.send_command(throttle=0.0)
            if time.monotonic() > deadline:
                raise RuntimeError('the flight controller did not arm '
                                   '(not ready? call wait_until_ready() first)')
            time.sleep(0.02)

    def disarm(self) -> None:
        """Stops the motors (the drone falls if it is in the air)."""
        with self._link.lock:
            self._link.arm_switch = False
            self._link.command = StickCommand()

    # --------------------------------------------------------------- timing

    def running(self) -> bool:
        """False once the arena run is over (time up)."""
        return not self.arena().finished and rclpy.ok()

    def sleep(self, seconds: float) -> None:
        """Waits `seconds` of simulation time (returns early if the run ends)."""
        end = self.time() + seconds
        while self.time() < end and self.running():
            time.sleep(0.002)

    def loop(self, hz: float) -> Iterator[int]:
        """Iterates at `hz` in simulation time until the run is over:
            for step in drone.loop(hz=20): ...
        """
        if not hz > 0.0:
            raise ValueError('hz must be > 0')
        period, step = 1.0 / hz, 0
        next_t = self.time()
        while self.running():
            yield step
            step += 1
            next_t += period
            while self.time() < next_t and self.running():
                time.sleep(0.001)

    def close(self) -> None:
        """Disarms and disconnects."""
        self.disarm()
        time.sleep(0.1)
        self._executor.shutdown()
        self._link.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
