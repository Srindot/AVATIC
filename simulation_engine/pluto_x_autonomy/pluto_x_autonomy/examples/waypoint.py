"""Example outer loop: fly a list of waypoints through MagisV2 angle mode.

Structure (all gains in params, SI units):

  horizontal  position error --P--> velocity setpoint (|v| <= max_speed)
              velocity error --PI--> acceleration setpoint (world ENU)
              acceleration --> tilt (a = g tan(tilt), |tilt| <= max_tilt),
              rotated into the heading frame --> roll/pitch sticks
              (stick = tilt / tilt_per_stick)
  vertical    altitude error --P--> climb-rate setpoint (|vz| <= max_climb)
              climb-rate error --PI--> throttle around hover_throttle,
              divided by cos(roll) cos(pitch) (tilt compensation)
  heading     yaw setpoint slews towards the waypoint heading at
              max_yaw_rate_deg_s; yaw-rate command = feed-forward slew rate
              + kp_yaw_1_s x yaw error, converted to the yaw stick with
              yaw_rate_per_stick_deg_s (stick + = clockwise)

Waypoints are [x, y, z] or [x, y, z, yaw_deg]: ENU metres relative to the
take-off point, and a heading in degrees relative to the take-off heading,
counter-clockwise positive (REP-103), UNWRAPPED: 360 means one full turn
to the left, so a camera pan is a waypoint with the same position and a
larger yaw. Without yaw_deg the previous heading is kept. A waypoint is
reached when the vehicle is within acceptance_radius_m, slower than
acceptance_speed_m_s and (if it has a heading) within acceptance_yaw_deg
of it, for hold_time_s; after the last one done() is True and the host
lands.

Stick-to-tilt scale: tilt_per_stick_deg is the steady bank the firmware
holds per unit stick. The default (32 deg) was measured in the simulator
(0.2 stick -> 6.5 deg firmware estimate); it is the small-stick slope
of the firmware's expo curve, which saturates at 20 deg from ~0.45 stick,
so keep tilt commands small; measure it on hardware too.
hover_throttle is only the starting point of the throttle integrator.

This is an example of the interface, not a tuned mission controller.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from ..api import (FcStatus, Observation, OuterLoopController, StickCommand,
                   wrap_angle)

GRAVITY_M_S2 = 9.81

DEFAULTS = {
    'waypoints_enu_m': [[0.0, 0.0, 1.0]],
    'acceptance_radius_m': 0.15,
    'acceptance_speed_m_s': 0.2,
    'hold_time_s': 1.0,
    'max_speed_m_s': 0.8,
    'max_tilt_deg': 15.0,
    'tilt_per_stick_deg': 32.0,
    'kp_position_1_s': 1.0,
    'kp_velocity_1_s': 2.0,
    'ki_velocity_1_s2': 0.3,
    'max_climb_m_s': 0.5,
    'kp_altitude_1_s': 1.5,
    'kp_climb_throttle_s_m': 0.25,
    'ki_climb_throttle_1_m': 0.15,
    'hover_throttle': 0.76,   # sim, with the 8 g camera module (1760 us)
    'max_yaw_rate_deg_s': 45.0,
    'yaw_rate_per_stick_deg_s': 77.0,   # measured in the sim (check_yaw.sh open_loop)
    'kp_yaw_1_s': 2.0,
    'acceptance_yaw_deg': 5.0,
}


def _limit_norm(vector: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit or norm == 0.0 else vector * (limit / norm)


class WaypointController(OuterLoopController):

    def __init__(self, params: Mapping[str, Any]):
        merged = dict(DEFAULTS)
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown WaypointController params: {sorted(unknown)}')
        merged.update(params)
        super().__init__(merged)
        p = self.params
        raw = [np.asarray(w, dtype=float) for w in p['waypoints_enu_m']]
        if not raw or any(w.shape not in ((3,), (4,)) for w in raw):
            raise ValueError('waypoints_enu_m must be a non-empty list of '
                             '[x, y, z] or [x, y, z, yaw_deg]')
        self.waypoints = [w[:3] for w in raw]
        # heading per waypoint, relative to take-off (rad); None = keep
        self.waypoint_yaw = [math.radians(w[3]) if w.shape == (4,) else None
                             for w in raw]
        self.yaw_unwrapped = 0.0
        self._last_yaw_obs = None
        self.yaw_start = 0.0
        self.yaw_sp = 0.0
        self.index = 0
        self.origin = np.zeros(3)
        self.velocity_integral = np.zeros(2)
        self.climb_integral = 0.0
        self.reached_since_s = None
        self._last = {}

    def reset(self, obs: Observation, status: FcStatus) -> None:
        self.origin = obs.position_enu_m.copy()
        self.yaw_start = obs.yaw_rad
        self.yaw_sp = obs.yaw_rad
        self.yaw_unwrapped = obs.yaw_rad
        self._last_yaw_obs = obs.yaw_rad
        self.index = 0
        self.velocity_integral[:] = 0.0
        self.climb_integral = 0.0
        self.reached_since_s = None

    def done(self) -> bool:
        return self.index >= len(self.waypoints)

    def _heading_target(self) -> float:
        """Absolute unwrapped yaw target of the current waypoint."""
        for i in range(min(self.index, len(self.waypoints) - 1), -1, -1):
            if self.waypoint_yaw[i] is not None:
                return self.yaw_start + self.waypoint_yaw[i]
        return self.yaw_start

    def current_target_enu_m(self) -> np.ndarray:
        return self.origin + self.waypoints[min(self.index, len(self.waypoints) - 1)]

    def update(self, obs: Observation, status: FcStatus,
               dt: float) -> StickCommand:
        p = self.params
        target = self.current_target_enu_m()
        error = target - obs.position_enu_m
        velocity = obs.velocity_enu_m_s

        # continuous (unwrapped) heading
        if self._last_yaw_obs is None:
            self._last_yaw_obs = self.yaw_unwrapped = obs.yaw_rad
        self.yaw_unwrapped += wrap_angle(obs.yaw_rad - self._last_yaw_obs)
        self._last_yaw_obs = obs.yaw_rad
        yaw_target = self._heading_target()

        # waypoint sequencing
        yaw_ok = (self.waypoint_yaw[min(self.index, len(self.waypoints) - 1)] is None
                  or abs(yaw_target - self.yaw_unwrapped)
                  < math.radians(p['acceptance_yaw_deg']))
        if (np.linalg.norm(error) < p['acceptance_radius_m'] and
                np.linalg.norm(velocity) < p['acceptance_speed_m_s'] and yaw_ok):
            if self.reached_since_s is None:
                self.reached_since_s = obs.time_s
            if obs.time_s - self.reached_since_s >= p['hold_time_s']:
                self.index += 1
                self.reached_since_s = None
        else:
            self.reached_since_s = None

        # horizontal: position P -> velocity PI -> acceleration -> tilt
        velocity_sp = _limit_norm(p['kp_position_1_s'] * error[:2],
                                  p['max_speed_m_s'])
        velocity_error = velocity_sp - velocity[:2]
        accel = (p['kp_velocity_1_s'] * velocity_error +
                 p['ki_velocity_1_s2'] * self.velocity_integral)
        max_tilt = math.radians(p['max_tilt_deg'])
        max_accel = GRAVITY_M_S2 * math.tan(max_tilt)
        if np.linalg.norm(accel) < max_accel:  # anti-windup: integrate unsaturated
            self.velocity_integral += velocity_error * dt
        accel = _limit_norm(accel, max_accel)

        # into the heading frame: forward = (cos yaw, sin yaw), left = (-sin, cos)
        cy, sy = math.cos(obs.yaw_rad), math.sin(obs.yaw_rad)
        accel_forward = cy * accel[0] + sy * accel[1]
        accel_left = -sy * accel[0] + cy * accel[1]
        tilt_forward = math.atan2(accel_forward, GRAVITY_M_S2)  # nose down
        tilt_right = math.atan2(-accel_left, GRAVITY_M_S2)      # bank right
        tilt_per_stick = math.radians(p['tilt_per_stick_deg'])

        # vertical: altitude P -> climb-rate PI -> throttle
        climb_sp = float(np.clip(p['kp_altitude_1_s'] * error[2],
                                 -p['max_climb_m_s'], p['max_climb_m_s']))
        climb_error = climb_sp - velocity[2]
        self.climb_integral += climb_error * dt
        throttle = (p['hover_throttle'] +
                    p['kp_climb_throttle_s_m'] * climb_error +
                    p['ki_climb_throttle_1_m'] * self.climb_integral)
        tilt_factor = max(math.cos(obs.roll_rad) * math.cos(obs.pitch_rad), 0.5)
        throttle /= tilt_factor
        if not 0.0 <= throttle <= 1.0:  # anti-windup
            self.climb_integral -= climb_error * dt
            throttle = min(max(throttle, 0.0), 1.0)

        # heading: slew the setpoint, rate = feed-forward + P on the error;
        # yaw stick + = clockwise = decreasing ENU yaw
        max_rate = math.radians(p['max_yaw_rate_deg_s'])
        step = float(np.clip(yaw_target - self.yaw_sp, -max_rate * dt, max_rate * dt))
        self.yaw_sp += step
        feedforward = step / dt if dt > 0.0 else 0.0
        rate_cmd = feedforward + p['kp_yaw_1_s'] * (self.yaw_sp - self.yaw_unwrapped)
        yaw_stick = -math.degrees(rate_cmd) / p['yaw_rate_per_stick_deg_s']

        self._last = {'wp': self.index, 'err_m': round(float(np.linalg.norm(error)), 3),
                      'yaw_err_deg': round(math.degrees(self.yaw_sp - self.yaw_unwrapped), 1),
                      'throttle': round(throttle, 3)}
        return StickCommand(roll=tilt_right / tilt_per_stick,
                            pitch=tilt_forward / tilt_per_stick,
                            yaw=yaw_stick, throttle=throttle)

    def diagnostics(self) -> Mapping[str, Any]:
        return self._last

    def setpoint_enu_m(self):
        return None if self.done() else self.current_target_enu_m()

    def setpoint_yaw_rad(self):
        return None if self.done() else self.yaw_sp
