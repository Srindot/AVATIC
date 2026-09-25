"""The outer-loop controller interface.

An outer-loop controller turns what the vehicle knows (an Observation and the
flight controller's status) into stick commands for the MagisV2 flight
controller, which runs the inner loops (attitude estimation, angle mode,
rate control, mixing). This is exactly the interface the real Pluto X
offers over its Wi-Fi link: RC channels in, telemetry out. A controller
written against it runs unchanged on the simulator and, through a hardware
bridge, on the vehicle.

To write one, subclass OuterLoopController and implement update():

    from pluto_x_autonomy.api import OuterLoopController, StickCommand

    class MyController(OuterLoopController):
        def __init__(self, params):
            super().__init__(params)
            self.target_z = float(params.get('altitude_m', 1.0))

        def update(self, obs, status, dt):
            ...
            return StickCommand(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.6)

and start the host with it (see host_node.py):

    ros2 launch pluto_x_bringup sim.launch.py \\
        controller:=my_package.my_module:MyController \\
        controller_params_file:=/path/params.yaml

The host arms the vehicle, calls reset() once when the flight starts, then
update() at a fixed rate of simulation time until done() returns True; it
then lands and disarms. Exceptions from the controller trigger the failsafe.

Conventions (REP-103):
  world  ENU: x east, y north, z up; origin = take-off point
  body   FLU: x forward, y left, z up
  yaw    rad, counter-clockwise from east (ENU); roll/pitch are ZYX Euler
         angles of the FLU body, i.e. roll + = right side DOWN, pitch + =
         nose DOWN (rotation about +y left)
"""

from __future__ import annotations

import abc
import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import numpy as np


@dataclass(frozen=True)
class Observation:
    """What the outer loop knows about the vehicle.

    source tells where it comes from: 'ground_truth' (simulator odometry,
    for development only: no real vehicle has it) or an estimator/camera
    pipeline name. Controllers meant for the real vehicle must not rely on
    ground truth.
    """

    time_s: float
    position_enu_m: np.ndarray
    velocity_enu_m_s: np.ndarray
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    angular_velocity_flu_rad_s: np.ndarray
    source: str = 'ground_truth'


@dataclass(frozen=True)
class FcStatus:
    """Flight-controller telemetry (its own estimates, not truth).

    Conventions as pluto_x_interfaces/FlightControllerStatus: roll + = right
    side down, pitch + = nose UP, heading clockwise from north in degrees.
    """

    time_s: float
    armed: bool
    ok_to_arm: bool
    calibrated: bool
    angle_mode: bool
    altitude_hold: bool
    healthy: bool
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    altitude_m: float
    battery_v: float


@dataclass(frozen=True)
class StickCommand:
    """Pilot-stick command, normalised.

    roll      [-1, 1]  +1 = full right (bank right)
    pitch     [-1, 1]  +1 = full forward (nose down, accelerate forward)
    yaw       [-1, 1]  +1 = full right (turn clockwise seen from above)
    throttle  [ 0, 1]  0 = minimum
    altitude_hold      engage the firmware's barometric altitude hold
                       (MagisV2 BARO mode, AUX3)

    In MagisV2 angle mode roll/pitch set an attitude setpoint and yaw sets a
    yaw rate; the stick-to-angle map depends on the firmware's rates and
    expo (measured in the simulator: 0.2 stick ~ 6.5 deg estimated bank).
    Values outside the range are clipped by the host.
    """

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    altitude_hold: bool = False

    def clipped(self) -> 'StickCommand':
        def clip(value: float, low: float, high: float) -> float:
            if not math.isfinite(value):
                raise ValueError(f'non-finite stick value {value}')
            return min(max(value, low), high)

        return StickCommand(roll=clip(self.roll, -1.0, 1.0),
                            pitch=clip(self.pitch, -1.0, 1.0),
                            yaw=clip(self.yaw, -1.0, 1.0),
                            throttle=clip(self.throttle, 0.0, 1.0),
                            altitude_hold=bool(self.altitude_hold))


class OuterLoopController(abc.ABC):
    """Base class of developer outer-loop controllers (simulator checks;
    participants use avatic_drone instead)."""

    def __init__(self, params: Mapping[str, Any]):
        self.params = dict(params)

    def reset(self, obs: Observation, status: FcStatus) -> None:
        """Called once, when the vehicle is armed and the flight starts."""

    @abc.abstractmethod
    def update(self, obs: Observation, status: FcStatus,
               dt: float) -> StickCommand:
        """Returns the stick command for this control step.

        dt is the simulation time since the previous call (the host's fixed
        period, except for the first call).
        """

    def done(self) -> bool:
        """True when the mission is complete; the host then lands."""
        return False

    def diagnostics(self) -> Mapping[str, Any]:
        """Optional key/values the host logs periodically."""
        return {}

    def setpoint_enu_m(self) -> Optional[np.ndarray]:
        """Optional: the position the controller is currently steering to
        (world ENU, m). The host publishes it on /pluto/outer_loop/setpoint
        so tracking error can be evaluated; None = not applicable."""
        return None

    def setpoint_yaw_rad(self) -> Optional[float]:
        """Optional: the heading the controller is steering to (ENU yaw,
        counter-clockwise from east, unwrapped). Published with the position
        setpoint; None = not applicable."""
        return None


def wrap_angle(angle_rad: float) -> float:
    """Wraps an angle to (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi
