"""Flight supervision around a participant outer-loop controller.

The supervisor owns everything a participant should not have to get right
to avoid crashing: waiting for the flight controller to calibrate, arming,
landing when the mission is over, and a failsafe when something breaks. It
is ROS-free (tested in isolation); host_node.py wires it to topics.

Phases:
  WAIT_LINK   no flight-controller status yet               sticks idle, disarmed
  WAIT_READY  status ok, waiting for ok_to_arm (calibration) sticks idle, disarmed
  ARMING      AUX4 on with throttle minimum, until armed    (arm_timeout_s)
  FLYING      controller.update() every step
  LANDING     vertical-speed controller on the observation, and horizontal
              position hold over the point where the landing started
              (position P + velocity D -> tilt -> sticks), until
              touchdown; then disarm
  FAILSAFE    level sticks, throttle ramps down from the hover estimate
              (no observation needed); disarm when the flight controller's
              altitude estimate is near zero or the throttle reaches zero
  DISARMED    mission over (final)
  ABORTED     could not arm/fly (final; reason in `reason`)

FAILSAFE is entered on: controller exception or invalid command, stale
observation or status, flight controller unhealthy, geofence violation.
If the flight controller disarms on its own while flying, the supervisor
stops in ABORTED.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .api import FcStatus, Observation, OuterLoopController, StickCommand
from .rc_mapping import IDLE


class Phase(enum.Enum):
    WAIT_LINK = 'wait_link'
    WAIT_READY = 'wait_ready'
    ARMING = 'arming'
    FLYING = 'flying'
    LANDING = 'landing'
    FAILSAFE = 'failsafe'
    DISARMED = 'disarmed'
    ABORTED = 'aborted'


@dataclass(frozen=True)
class SupervisorParams:
    auto_arm: bool = True
    ready_timeout_s: float = 120.0
    arm_timeout_s: float = 3.0
    observation_timeout_s: float = 0.5
    status_timeout_s: float = 0.5
    # geofence relative to the take-off point
    max_altitude_m: float = 5.0
    max_horizontal_distance_m: float = 20.0
    # landing (vertical-speed PI on the observation)
    land_descent_speed_m_s: float = 0.3
    land_kp_throttle_per_m_s: float = 0.15
    land_ki_throttle_per_m: float = 0.10
    land_hold_position: bool = True
    land_kp_position_1_s2: float = 1.0      # accel per m of position error
    land_kd_velocity_1_s: float = 1.5       # accel per m/s of velocity
    land_max_tilt_deg: float = 10.0
    land_tilt_per_stick_deg: float = 32.0   # measured in the sim (see waypoint.py)
    touchdown_height_m: float = 0.05
    touchdown_speed_m_s: float = 0.1
    touchdown_time_s: float = 0.5
    land_timeout_s: float = 60.0
    # failsafe (open loop)
    failsafe_throttle_scale: float = 0.95
    failsafe_throttle_ramp_per_s: float = 0.05
    failsafe_disarm_altitude_m: float = 0.15
    # hover-throttle estimate used by LANDING and FAILSAFE: initial guess,
    # then a low-pass of the throttle while FLYING
    hover_throttle_guess: float = 0.76
    hover_throttle_filter_s: float = 2.0


class Supervisor:

    def __init__(self, controller: OuterLoopController,
                 params: SupervisorParams = SupervisorParams()):
        self.controller = controller
        self.params = params
        self.phase = Phase.WAIT_LINK
        self.reason = ''
        self._phase_start_s: Optional[float] = None
        self._last_time_s: Optional[float] = None
        self._origin = None
        self._hover_throttle = params.hover_throttle_guess
        self._land_integral = 0.0
        self._touchdown_since_s: Optional[float] = None
        self._failsafe_throttle = 0.0
        self._last_flying_time_s: Optional[float] = None
        self._land_xy = None

    # -- helpers ---------------------------------------------------------

    def _enter(self, phase: Phase, time_s: float, reason: str = '') -> None:
        self.phase = phase
        self._phase_start_s = time_s
        if reason:
            self.reason = reason

    def _in_phase_s(self, time_s: float) -> float:
        return time_s - (self._phase_start_s if self._phase_start_s is not None
                         else time_s)

    def _enter_failsafe(self, time_s: float, reason: str) -> None:
        self._failsafe_throttle = (self._hover_throttle *
                                   self.params.failsafe_throttle_scale)
        self._enter(Phase.FAILSAFE, time_s, reason)

    @staticmethod
    def _stale(sample_time_s: Optional[float], now_s: float,
               timeout_s: float) -> bool:
        return sample_time_s is None or now_s - sample_time_s > timeout_s

    def _geofence_violation(self, obs: Observation) -> Optional[str]:
        if self._origin is None:
            return None
        offset = obs.position_enu_m - self._origin
        if offset[2] > self.params.max_altitude_m:
            return f'geofence: altitude {offset[2]:.2f} m'
        if math.hypot(offset[0], offset[1]) > \
                self.params.max_horizontal_distance_m:
            return 'geofence: horizontal distance'
        return None

    # -- main step -------------------------------------------------------

    def step(self, time_s: float, obs: Optional[Observation],
             status: Optional[FcStatus]) -> Tuple[StickCommand, bool]:
        """Returns (stick command, arm switch) for this step."""
        dt = 0.0 if self._last_time_s is None else time_s - self._last_time_s
        self._last_time_s = time_s
        p = self.params
        status_ok = status is not None and not self._stale(
            status.time_s, time_s, p.status_timeout_s)
        obs_ok = obs is not None and not self._stale(
            obs.time_s, time_s, p.observation_timeout_s)

        if self.phase == Phase.WAIT_LINK:
            if self._phase_start_s is None:
                self._phase_start_s = time_s
            if status_ok:
                self._enter(Phase.WAIT_READY, time_s)
            return IDLE, False

        if self.phase == Phase.WAIT_READY:
            if status_ok and status.ok_to_arm and obs_ok and p.auto_arm:
                self._enter(Phase.ARMING, time_s)
            elif self._in_phase_s(time_s) > p.ready_timeout_s:
                self._enter(Phase.ABORTED, time_s,
                            'flight controller never reported ok_to_arm')
            return IDLE, False

        if self.phase == Phase.ARMING:
            if status_ok and status.armed:
                self._origin = obs.position_enu_m.copy() if obs_ok else None
                try:
                    self.controller.reset(obs, status)
                except Exception as error:  # noqa: BLE001 (participant code)
                    self._enter(Phase.ABORTED, time_s,
                                f'controller reset() raised: {error!r}')
                    return IDLE, False
                self._enter(Phase.FLYING, time_s)
                self._last_flying_time_s = None
            elif self._in_phase_s(time_s) > p.arm_timeout_s:
                self._enter(Phase.ABORTED, time_s, 'arming timed out')
                return IDLE, False
            return IDLE, True

        if self.phase == Phase.FLYING:
            if not status_ok:
                self._enter_failsafe(time_s, 'flight-controller status stale')
            elif not status.armed:
                self._enter(Phase.ABORTED, time_s,
                            'flight controller disarmed in flight')
                return IDLE, False
            elif not status.healthy:
                self._enter_failsafe(time_s, 'flight controller unhealthy')
            elif not obs_ok:
                self._enter_failsafe(time_s, 'observation stale')
            else:
                violation = self._geofence_violation(obs)
                if violation:
                    self._enter_failsafe(time_s, violation)
            if self.phase == Phase.FLYING:
                if self.controller.done():
                    self._land_integral = 0.0
                    self._touchdown_since_s = None
                    self._land_xy = obs.position_enu_m[:2].copy()
                    self._enter(Phase.LANDING, time_s, 'mission complete')
                else:
                    step_dt = (dt if self._last_flying_time_s is not None
                               else 0.0)
                    self._last_flying_time_s = time_s
                    try:
                        command = self.controller.update(obs, status, step_dt)
                        if not isinstance(command, StickCommand):
                            raise TypeError(
                                f'update() returned {type(command).__name__},'
                                ' expected StickCommand')
                        command = command.clipped()
                    except Exception as error:  # noqa: BLE001
                        self._enter_failsafe(
                            time_s, f'controller update() raised: {error!r}')
                    else:
                        if step_dt > 0.0 and not command.altitude_hold:
                            alpha = min(step_dt / p.hover_throttle_filter_s, 1.0)
                            self._hover_throttle += alpha * (
                                command.throttle - self._hover_throttle)
                        return command, True

        if self.phase == Phase.LANDING:
            return self._land(time_s, dt, obs if obs_ok else None,
                              status if status_ok else None)

        if self.phase == Phase.FAILSAFE:
            return self._failsafe(dt, time_s, status if status_ok else None)

        return IDLE, False  # DISARMED, ABORTED

    def _land(self, time_s: float, dt: float, obs: Optional[Observation],
              status: Optional[FcStatus]) -> Tuple[StickCommand, bool]:
        p = self.params
        if obs is None or status is None or not status.armed:
            if status is not None and not status.armed:
                self._enter(Phase.DISARMED, time_s, self.reason)
                return IDLE, False
            self._enter_failsafe(time_s, 'lost observation while landing')
            return self._failsafe(0.0, time_s, status)
        if self._in_phase_s(time_s) > p.land_timeout_s:
            self._enter_failsafe(time_s, 'landing timed out')
            return self._failsafe(0.0, time_s, status)
        height = obs.position_enu_m[2] - (self._origin[2]
                                          if self._origin is not None else 0.0)
        vz = obs.velocity_enu_m_s[2]
        if height < p.touchdown_height_m and abs(vz) < p.touchdown_speed_m_s:
            if self._touchdown_since_s is None:
                self._touchdown_since_s = time_s
            if time_s - self._touchdown_since_s >= p.touchdown_time_s:
                self._enter(Phase.DISARMED, time_s, self.reason)
                return IDLE, False
        else:
            self._touchdown_since_s = None
        error = -p.land_descent_speed_m_s - vz
        self._land_integral += error * dt
        throttle = (self._hover_throttle + p.land_kp_throttle_per_m_s * error +
                    p.land_ki_throttle_per_m * self._land_integral)
        if self._touchdown_since_s is not None:
            return StickCommand(throttle=0.0), True  # on the ground: spin down
        roll = pitch = 0.0
        if p.land_hold_position and self._land_xy is not None:
            accel = (p.land_kp_position_1_s2 * (self._land_xy - obs.position_enu_m[:2])
                     - p.land_kd_velocity_1_s * obs.velocity_enu_m_s[:2])
            max_accel = 9.81 * math.tan(math.radians(p.land_max_tilt_deg))
            norm = float(math.hypot(accel[0], accel[1]))
            if norm > max_accel:
                accel = accel * (max_accel / norm)
            cy, sy = math.cos(obs.yaw_rad), math.sin(obs.yaw_rad)
            forward = cy * accel[0] + sy * accel[1]
            left = -sy * accel[0] + cy * accel[1]
            per_stick = math.radians(p.land_tilt_per_stick_deg)
            pitch = math.atan2(forward, 9.81) / per_stick
            roll = math.atan2(-left, 9.81) / per_stick
        return StickCommand(roll=roll, pitch=pitch, throttle=throttle).clipped(), True

    def _failsafe(self, dt: float, time_s: float,
                  status: Optional[FcStatus]) -> Tuple[StickCommand, bool]:
        p = self.params
        self._failsafe_throttle = max(
            self._failsafe_throttle - p.failsafe_throttle_ramp_per_s * dt, 0.0)
        near_ground = (status is not None and
                       status.altitude_m < p.failsafe_disarm_altitude_m and
                       self._in_phase_s(time_s) > 1.0)
        if self._failsafe_throttle <= 0.0 or near_ground:
            self._enter(Phase.DISARMED, time_s, self.reason)
            return IDLE, False
        return StickCommand(throttle=self._failsafe_throttle).clipped(), True

    @property
    def hover_throttle_estimate(self) -> float:
        return self._hover_throttle

    @property
    def finished(self) -> bool:
        return self.phase in (Phase.DISARMED, Phase.ABORTED)
