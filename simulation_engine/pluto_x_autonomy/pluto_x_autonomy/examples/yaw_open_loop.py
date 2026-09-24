"""Test controller: open-loop yaw-stick steps while holding position.

Characterises the yaw axis of the vehicle + MagisV2 (stick -> yaw rate,
response time, overshoot, coupling into position and altitude). Horizontal
position and altitude are held by the WaypointController loops; the yaw
stick follows a fixed schedule instead of the heading loop.

params:
  hover_enu_m     [x, y, z] relative to take-off (default [0, 0, 1])
  settle_s        hover time before the first step (default 5)
  steps           list of [yaw_stick, duration_s]; stick + = clockwise
"""

from __future__ import annotations

from typing import Any, Mapping

from ..api import StickCommand
from .waypoint import WaypointController

DEFAULT_STEPS = [[0.1, 4.0], [0.0, 3.0], [0.2, 4.0], [0.0, 3.0],
                 [0.4, 4.0], [0.0, 3.0], [-0.4, 4.0], [0.0, 3.0],
                 [1.0, 3.0], [0.0, 4.0]]


class YawOpenLoopTest(WaypointController):

    def __init__(self, params: Mapping[str, Any]):
        own = dict(params)
        self.hover = list(own.pop('hover_enu_m', [0.0, 0.0, 1.0]))
        self.settle_s = float(own.pop('settle_s', 5.0))
        self.steps = [(float(s), float(d)) for s, d in own.pop('steps', DEFAULT_STEPS)]
        own['waypoints_enu_m'] = [self.hover]
        super().__init__(own)
        self.t = 0.0
        self.stick = 0.0

    def update(self, obs, status, dt):
        self.t += dt
        base = super().update(obs, status, dt)
        self.index = 0  # never advance: hold the hover point
        elapsed = self.t - self.settle_s
        self.stick = None
        for stick, duration in self.steps:
            if 0.0 <= elapsed < duration:
                self.stick = stick
                break
            elapsed -= duration
        if self.stick is None:  # before the schedule: heading hold
            return base
        return StickCommand(roll=base.roll, pitch=base.pitch, yaw=self.stick,
                            throttle=base.throttle)

    def done(self):
        return self.t > self.settle_s + sum(d for _, d in self.steps)

    def setpoint_yaw_rad(self):
        return None  # open loop

    def diagnostics(self):
        return {**super().diagnostics(), 'yaw_stick': self.stick}
