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
    drone.close()                    # sim: disarms; real drone: lands, then disarms

What the drone understands (MagisV2 angle mode, as on the real Pluto X):

  roll      -1..1  bank angle setpoint;  + = bank right   (~7 deg at 0.2,
                                          ~16 at 0.4, 20 deg max from ~0.45)
  pitch     -1..1  tilt angle setpoint;  + = nose down = move forward
  yaw_rate  -1..1  turn rate;            + = clockwise    (~77 deg/s per unit;
                                          capped at 0.8, ~62 deg/s)
  throttle   0..1  collective thrust;    hover ~0.76 (depends on battery)

roll and pitch are ANGLES the firmware holds, not rates; only yaw is a
rate. The numbers in brackets were measured in the simulator.

Safety caps (both backends, avatic_drone.types.SafetyLimits): |roll|,
|pitch| <= 0.6, |yaw_rate| <= 0.8, throttle <= 0.95, and above 2.5 m
(barometric) the throttle is limited below hover (0.9 x the hover
estimate, itself at most 0.8) so the drone descends.

What you get is only what the real drone provides: the camera image and
the flight controller's own estimates (attitude, heading, barometric
altitude, battery). There is no position or velocity.

All times are SIMULATION time (seconds).
"""

from __future__ import annotations

import math
import threading
import time
from typing import Iterator, Optional

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

from .types import (SAFETY_LIMITS, ArenaStatus, Command, Frame, Telemetry,
                    apply_safety_limits)

COMMAND_RATE_HZ = 50.0        # stick commands are streamed at the MSP rate
COMMAND_TIMEOUT_S = 0.5       # no send_command() for this long -> failsafe (see below)
# failsafe throttle: a running average of the throttle sent (~hover when
# flying level), limited to this range
HOVER_ESTIMATE_TIME_CONSTANT_S = 2.0
FAILSAFE_THROTTLE_RANGE = (0.6, 0.85)
DEFAULT_HOVER_THROTTLE = 0.76
CEILING_HOVER_FRACTION = 0.9     # above the ceiling otherwise: 0.9 x the hover estimate,
CEILING_HOVER_MAX = 0.8          # ... the estimate capped (a climbing controller inflates it)
AIRBORNE_ALT_M = 0.3          # the hover estimate learns only above this altitude
FLYING_THROTTLE = 0.3         # failsafe: above this the drone is assumed airborne
ARM_TIMEOUT_S = 10.0          # sim time runs slower than real time on slow PCs


class _Link(Node):
    """ROS side: subscriptions, the command stream. Runs in a thread."""

    def __init__(self):
        super().__init__('avatic_drone', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.image_msg = None
        self.image_seq = 0
        self.status = None
        self.arena = ArenaStatus()
        self.command = StickCommand()
        self.arm_switch = False
        self.last_command_s = None
        self.timed_out = False
        self.hover_estimate = DEFAULT_HOVER_THROTTLE
        self.warned_caps = set()
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
            self.image_seq += 1

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
            altitude_m = self.status.altitude_m if self.status is not None else None
            if (arm and self.last_command_s is not None and
                    now - self.last_command_s > COMMAND_TIMEOUT_S):
                # same rule as the hardware backend: never lift a grounded drone
                airborne = (command.throttle > FLYING_THROTTLE or
                            (altitude_m is not None and altitude_m > 0.3))
                if airborne:
                    low, high = FAILSAFE_THROTTLE_RANGE
                    safe = StickCommand(throttle=min(max(self.hover_estimate, low), high))
                    what = f'throttle {safe.throttle:.2f} (estimated hover)'
                else:
                    safe, what = StickCommand(throttle=0.0), 'throttle 0 (was on the ground)'
                if not self.timed_out:
                    self.get_logger().warn(
                        f'no send_command() for {COMMAND_TIMEOUT_S} s: failsafe - sticks level, '
                        f'no yaw, {what}')
                    self.timed_out = True
                command = safe
            elif (arm and altitude_m is not None and
                  AIRBORNE_ALT_M < altitude_m <= SAFETY_LIMITS.max_altitude_m):
                # learns only while flying (not from idle on the pad)
                # running average of the throttle sent while armed: ~hover
                alpha = min((1.0 / COMMAND_RATE_HZ) / HOVER_ESTIMATE_TIME_CONSTANT_S, 1.0)
                low, high = FAILSAFE_THROTTLE_RANGE
                self.hover_estimate = min(max(
                    self.hover_estimate + alpha * (command.throttle - self.hover_estimate),
                    low), high)
            # altitude ceiling (safety): above it, a throttle below hover so
            # the drone descends (same rule as the hardware backend)
            ceiling = CEILING_HOVER_FRACTION * min(self.hover_estimate, CEILING_HOVER_MAX)
            if (arm and altitude_m is not None and altitude_m > SAFETY_LIMITS.max_altitude_m
                    and command.throttle > ceiling):
                command = StickCommand(roll=command.roll, pitch=command.pitch, yaw=command.yaw,
                                       throttle=ceiling)
                if 'ceiling' not in self.warned_caps:
                    self.warned_caps.add('ceiling')
                    self.get_logger().warn(
                        f'above the {SAFETY_LIMITS.max_altitude_m} m safety ceiling: '
                        'throttle limited, descending (warned once)')
        rc = to_rc(command, arm)
        msg = RcCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        (msg.roll_us, msg.pitch_us, msg.throttle_us, msg.yaw_us, msg.aux1_us,
         msg.aux2_us, msg.aux3_us, msg.aux4_us) = rc.as_list()
        self.rc_pub.publish(msg)


class SimDrone:
    """Connection to the simulated Pluto X (see the module docstring)."""

    def __init__(self, connect_timeout_s: float = 60.0):
        self.closed = False
        if not rclpy.ok():
            rclpy.init()
        self._link = _Link()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._link)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        start = time.monotonic()
        deadline = start + connect_timeout_s
        hinted = False
        clock_seen = (0.0, start)   # (sim time, wall time): is the clock moving?
        paused = False
        while self._link.now_s() <= 0.0 or self._link.status is None:
            now_sim = self._link.now_s()
            if now_sim != clock_seen[0]:
                clock_seen = (now_sim, time.monotonic())
            # a clock that stopped: the simulator is paused (its run is over)
            paused = now_sim > 0.0 and time.monotonic() - clock_seen[1] > 3.0
            if paused and self._link.status is None:
                break
            if not hinted and time.monotonic() - start > 10.0:
                what = ('no simulation clock yet (is the simulator running?)'
                        if now_sim <= 0.0 else
                        'clock running but no flight-controller telemetry yet')
                print(f'[avatic_drone] still waiting for the simulator: {what}', flush=True)
                hinted = True
            if time.monotonic() > deadline:
                break
            time.sleep(0.05)
        if self._link.now_s() <= 0.0 or self._link.status is None:
            self._shutdown()  # a leaked node would keep streaming "disarm" later
            if paused:
                raise RuntimeError(
                    'the simulator is paused: its run is over (one run per launch). '
                    'Stop the launch (Ctrl-C) and start it again.')
            raise TimeoutError(
                'no simulator found: is it running? '
                '(ros2 launch pluto_x_bringup competition.launch.py)')

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races
            pass

    # ---------------------------------------------------------------- inputs

    def get_frame(self) -> Optional[Frame]:
        """Latest camera image (RGB, H x W x 3 uint8), or None before the first."""
        with self._link.lock:
            msg, seq = self._link.image_msg, self._link.image_seq
        if msg is None:
            return None
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError(f'unexpected camera encoding {msg.encoding!r}')
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)  # drop row padding
        if msg.encoding == 'bgr8':
            image = image[..., ::-1]
        return Frame(image=np.array(image, copy=True), seq=seq,  # own, writable copy
                     time_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
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
                         battery_v=s.battery_v)

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
                     yaw_rate: float = 0.0, throttle: float = 0.0) -> None:
        """Stick command, held until the next call (see the module docstring
        for the meaning and range of each value; out-of-range values are
        clipped). Call it at least every 0.5 s: otherwise a failsafe levels
        the sticks, stops the yaw and sets the throttle to an estimate of
        hover (the average of your recent throttle; 0 if the drone was on
        the ground) until the next call."""
        for name, value in (('roll', roll), ('pitch', pitch), ('yaw_rate', yaw_rate),
                            ('throttle', throttle)):
            if not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number, got {value}')
        safe, capped = apply_safety_limits(Command(roll, pitch, yaw_rate, throttle))
        for name in capped:
            if name not in self._link.warned_caps:
                self._link.warned_caps.add(name)
                self._link.get_logger().warn(
                    f'{name} command beyond the safety cap: clipped (see avatic_drone.types.'
                    'SafetyLimits); warned once')
        command = StickCommand(roll=safe.roll, pitch=safe.pitch, yaw=safe.yaw_rate,
                               throttle=safe.throttle).clipped()
        with self._link.lock:
            self._link.command = command
            self._link.last_command_s = self._link.now_s()
            self._link.timed_out = False

    def send(self, command: Command) -> None:
        """Sends a Command (same as send_command with its fields)."""
        if not isinstance(command, Command):
            raise TypeError(f'send() needs a Command, got {type(command).__name__}')
        self.send_command(roll=command.roll, pitch=command.pitch, yaw_rate=command.yaw_rate,
                          throttle=command.throttle)

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
        deadline = time.monotonic() + ARM_TIMEOUT_S
        while not getattr(self.get_telemetry(), 'armed', False):
            self.send_command(throttle=0.0)
            if time.monotonic() > deadline:
                with self._link.lock:
                    self._link.arm_switch = False
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
        warned = False
        while self.running():
            yield step
            step += 1
            next_t += period
            now = self.time()
            if now > next_t + period:  # the body took longer than a period:
                missed = int((now - next_t) / period)
                next_t += missed * period  # skip, don't burst to catch up
                if not warned:
                    print(f'[avatic_drone] loop({hz:g} Hz): an iteration took longer than '
                          f'{period * 1000:.0f} ms of sim time; skipping missed periods')
                    warned = True
            while self.time() < next_t and self.running():
                time.sleep(0.001)

    def close(self) -> None:
        """Disarms and disconnects."""
        global _SHARED
        if self.closed:
            return
        self.closed = True
        if _SHARED is self:
            _SHARED = None
        self.disarm()
        # let the disarm command go out (sim time may run slower than real time)
        deadline = time.monotonic() + 2.0
        while getattr(self.get_telemetry(), 'armed', False) and time.monotonic() < deadline:
            time.sleep(0.02)
        self._shutdown()

    def _shutdown(self):
        self.closed = True
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=2.0)  # stop the ROS thread before shutdown
        self._link.destroy_node()
        rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


_SHARED: Optional[SimDrone] = None


def connect() -> SimDrone:
    """The shared Drone for interactive use (Jupyter): re-running
    `drone = connect()` returns the same open connection instead of
    creating a second one (two Drones would fight over the drone)."""
    global _SHARED
    if _SHARED is None or _SHARED.closed:
        _SHARED = SimDrone()
    return _SHARED
