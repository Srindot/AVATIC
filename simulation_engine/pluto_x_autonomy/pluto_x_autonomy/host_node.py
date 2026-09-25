"""outer_loop_host: runs a developer OuterLoopController on the Pluto X
(simulator validation and the check scripts; not the participant path).

  observation   /sim/pluto/odometry (nav_msgs/Odometry), simulator ground
                truth, when observation_source = ground_truth (development
                only; the real vehicle has no such topic)
  status        /pluto/fc_status (pluto_x_interfaces/FlightControllerStatus)
  command       /pluto/rc (pluto_x_interfaces/RcCommand), every period

The control step runs on a timer of the node clock; launch it with
use_sim_time:=true so the step is fixed in SIMULATION time regardless of
the real-time factor.

Parameters:
  controller               'package.module:Class' or '/file.py:Class'
                           (default: the example WaypointController)
  controller_params_file   YAML mapping passed to the constructor ('' = {})
  rate_hz                  control rate (default 50, the MSP RC rate)
  observation_source       ground_truth (only option for now)
  odometry_topic, status_topic, rc_topic
  log_period_s             period of the status log line (default 1.0)
  shutdown_when_finished   exit the node after DISARMED/ABORTED (default true)
  supervisor.*             SupervisorParams fields (e.g. supervisor.
                           max_altitude_m)
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile

from pluto_x_interfaces.msg import (FlightControllerStatus, OuterLoopSetpoint,
                                    RcCommand)

from .api import FcStatus, Observation
from .loader import make_controller
from .rc_mapping import to_rc
from .supervisor import Phase, Supervisor, SupervisorParams

DEFAULT_CONTROLLER = 'pluto_x_autonomy.examples.waypoint:WaypointController'


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _euler_zyx_from_quaternion(x: float, y: float, z: float, w: float):
    """(roll, pitch, yaw) of a body-to-world quaternion (REP-103)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def observation_from_odometry(msg: Odometry) -> Observation:
    """Odometry (pose in world ENU, twist in the child/body FLU frame, as
    gz-sim's OdometryPublisher reports it) -> Observation."""
    q = msg.pose.pose.orientation
    roll, pitch, yaw = _euler_zyx_from_quaternion(q.x, q.y, q.z, q.w)
    # body -> world rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    v_body = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                       msg.twist.twist.linear.z])
    w_body = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                       msg.twist.twist.angular.z])
    p = msg.pose.pose.position
    return Observation(time_s=_stamp_s(msg.header.stamp),
                       position_enu_m=np.array([p.x, p.y, p.z]),
                       velocity_enu_m_s=rotation @ v_body,
                       roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
                       angular_velocity_flu_rad_s=w_body,
                       source='ground_truth')


def status_from_msg(msg: FlightControllerStatus) -> FcStatus:
    return FcStatus(time_s=_stamp_s(msg.header.stamp), armed=msg.armed,
                    ok_to_arm=msg.ok_to_arm, calibrated=msg.calibrated,
                    angle_mode=msg.angle_mode, altitude_hold=msg.altitude_hold,
                    healthy=msg.healthy, roll_deg=msg.roll_deg,
                    pitch_deg=msg.pitch_deg, heading_deg=msg.heading_deg,
                    altitude_m=msg.altitude_m, battery_v=msg.battery_v)


class OuterLoopHost(Node):

    def __init__(self):
        super().__init__('outer_loop_host')
        spec = self.declare_parameter('controller', DEFAULT_CONTROLLER).value
        params_file = self.declare_parameter('controller_params_file', '').value
        rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        source = self.declare_parameter('observation_source', 'ground_truth').value
        odom_topic = self.declare_parameter('odometry_topic',
                                            '/sim/pluto/odometry').value
        status_topic = self.declare_parameter('status_topic',
                                              '/pluto/fc_status').value
        rc_topic = self.declare_parameter('rc_topic', '/pluto/rc').value
        self._log_period_s = float(self.declare_parameter('log_period_s', 1.0).value)
        self._shutdown_when_finished = bool(
            self.declare_parameter('shutdown_when_finished', True).value)
        if not rate_hz > 0.0:
            raise ValueError('rate_hz must be > 0')
        if source != 'ground_truth':
            raise ValueError(f"observation_source '{source}' not supported yet")

        defaults = SupervisorParams()
        overrides = {}
        for f in dataclasses.fields(SupervisorParams):
            value = self.declare_parameter(f'supervisor.{f.name}',
                                           getattr(defaults, f.name)).value
            overrides[f.name] = type(getattr(defaults, f.name))(value)
        controller = make_controller(spec, params_file)
        self.supervisor = Supervisor(controller, SupervisorParams(**overrides))

        qos = QoSProfile(depth=10)
        self._obs = None
        self._status = None
        self.create_subscription(Odometry, odom_topic, self._on_odometry, qos)
        self.create_subscription(FlightControllerStatus, status_topic,
                                 self._on_status, qos)
        self._rc_pub = self.create_publisher(RcCommand, rc_topic, qos)
        self._setpoint_pub = self.create_publisher(
            OuterLoopSetpoint, '/pluto/outer_loop/setpoint', qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._step)
        self._last_phase = None
        self._next_log_s = 0.0
        self.get_logger().info(
            f"controller {spec} ({type(controller).__name__}), params "
            f"'{params_file or '{}'}', {rate_hz:g} Hz, observation {source} "
            f'[{odom_topic}]; status {status_topic}; RC {rc_topic}')
        if source == 'ground_truth':
            self.get_logger().warn(
                'observation_source=ground_truth: simulator truth, development '
                'only - no real vehicle provides it')

    def _publish_setpoint(self, stamp) -> None:
        controller = self.supervisor.controller
        try:  # optional participant methods; never fail the flight
            position = controller.setpoint_enu_m()
            yaw = controller.setpoint_yaw_rad()
        except Exception:  # noqa: BLE001
            return
        if position is None and yaw is None:
            return
        sp = OuterLoopSetpoint()
        sp.header.stamp = stamp
        sp.header.frame_id = 'world'
        if position is not None:
            sp.has_position = True
            sp.x_enu_m, sp.y_enu_m, sp.z_enu_m = (float(v) for v in position)
        if yaw is not None:
            sp.has_yaw = True
            sp.yaw_rad = float(yaw)
        self._setpoint_pub.publish(sp)

    def _on_odometry(self, msg: Odometry) -> None:
        self._obs = observation_from_odometry(msg)

    def _on_status(self, msg: FlightControllerStatus) -> None:
        self._status = status_from_msg(msg)

    def _step(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s <= 0.0:
            return  # no /clock yet
        command, arm = self.supervisor.step(now_s, self._obs, self._status)
        rc = to_rc(command, arm)
        msg = RcCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        (msg.roll_us, msg.pitch_us, msg.throttle_us, msg.yaw_us, msg.aux1_us,
         msg.aux2_us, msg.aux3_us, msg.aux4_us) = rc.as_list()
        self._rc_pub.publish(msg)
        if self.supervisor.phase == Phase.FLYING:
            self._publish_setpoint(msg.header.stamp)

        phase = self.supervisor.phase
        if phase != self._last_phase:
            reason = self.supervisor.reason
            log = (self.get_logger().error if phase in (Phase.ABORTED,
                                                        Phase.FAILSAFE)
                   else self.get_logger().info)
            log(f'[t={now_s:.2f} s] phase -> {phase.value}'
                + (f' ({reason})' if reason else ''))
            self._last_phase = phase
        if now_s >= self._next_log_s:
            self._next_log_s = now_s + self._log_period_s
            if phase == Phase.FLYING:
                diag = dict(self.supervisor.controller.diagnostics())
                pos = self._obs.position_enu_m if self._obs else None
                self.get_logger().info(
                    f'[t={now_s:.2f} s] pos_enu={np.round(pos, 2) if pos is not None else None} '
                    f'rc={rc.as_list()[:4]} {diag}')
        if self.supervisor.finished and self._shutdown_when_finished:
            self.get_logger().info(
                f'finished: {phase.value} ({self.supervisor.reason})')
            self._timer.cancel()
            raise SystemExit(0 if phase == Phase.DISARMED else 1)


def main(args=None):
    rclpy.init(args=args)
    node = OuterLoopHost()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exit_request:
        code = int(exit_request.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
