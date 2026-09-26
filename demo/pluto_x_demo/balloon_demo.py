"""WORKSHOP DEMO: pop balloons along a hand-planned route.

THIS IS NOT A SOLUTION TO THE CHALLENGE. It reads the balloon positions
from the arena configuration and flies to them using simulator ground
truth, neither of which a team has on the real drone. Its purpose is to
show the arena mechanics (popping, points, the time limit) and what the
vehicle can do in a short run. A competition entry has to find and approach the
balloons from the camera image (visual servoing).

Behaviour: first a vertical TAKE-OFF to takeoff_height_m with level
sticks and the heading held (commanding tilt or yaw while the drone sits on
the ground only presses it into the ground: the firmware's attitude loop
saturates and the collective thrust never exceeds the weight). Then, per
balloon on the route:
  1. TURN: hold position (climbing to the balloon's height) and yaw until
     the nose points at the balloon
  2. APPROACH: fly at the balloon's centre while keeping the nose on it
     (the heading setpoint is the bearing to the balloon, every step)
  3. the balloon pops on first contact (arena plugin); the controller
     moves on when its reference point is within pop_distance_m of the
     centre (the contact sphere has touched the surface by then)

The position/altitude/yaw loops are those of WaypointController.

params (config/balloon_demo.yaml):
  arena_config     arena YAML with the balloon positions ('' = the default
                   pluto_x_gazebo config/arena_default.yaml)
  route            balloon indices (order of the arena's `balloons` list)
  takeoff_height_m climb straight up to this height first
  face_tolerance_deg   start the approach when the heading error is below
  pop_distance_m   consider the balloon popped inside this distance
  plus any WaypointController gain (max_speed_m_s, max_tilt_deg, ...)
"""

from __future__ import annotations

import math
import os
from typing import Any, List, Mapping

import numpy as np
import yaml

from pluto_x_autonomy.api import FcStatus, Observation, StickCommand, wrap_angle
from pluto_x_autonomy.examples.waypoint import WaypointController

OWN_DEFAULTS = {
    'arena_config': '',
    'route': [0],
    'takeoff_height_m': 0.5,
    'face_tolerance_deg': 20.0,
    'pop_distance_m': 0.20,
    'heading_freeze_distance_m': 0.35,
}


def default_arena_config() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('pluto_x_gazebo'),
                        'config', 'arena_default.yaml')


def load_balloons(path: str) -> List[dict]:
    with open(path, 'r', encoding='utf-8') as stream:
        arena = yaml.safe_load(stream)
    balloons = []
    for i, b in enumerate(arena['balloons']):
        balloons.append({
            'index': i,
            'name': f"balloon_{i}_{b['color']}",  # as the arena plugin names them
            'color': b['color'],
            'points': int(arena['colors'][b['color']]['points']),
            'position': np.asarray(b['position_enu_m'], dtype=float),
        })
    return balloons


class BalloonDemoController(WaypointController):

    def __init__(self, params: Mapping[str, Any]):
        own = dict(OWN_DEFAULTS)
        rest = {}
        for key, value in params.items():
            (own if key in OWN_DEFAULTS else rest)[key] = value
        rest.setdefault('waypoints_enu_m', [[0.0, 0.0, 1.0]])  # unused placeholder
        super().__init__(rest)
        self.demo = own
        path = own['arena_config'] or default_arena_config()
        balloons = load_balloons(path)
        try:
            self.route = [balloons[int(i)] for i in own['route']]
        except (IndexError, ValueError) as error:
            raise ValueError(f'route {own["route"]} does not match {path}') from error
        if not self.route:
            raise ValueError('route is empty')
        self.leg = 0
        self.phase = 'takeoff'
        self.takeoff_z = 0.0
        self.hold_xy = np.zeros(2)
        self.log = []  # (time, event) for diagnostics
        self._obs = None

    # -- helpers ---------------------------------------------------------

    @property
    def target(self):
        return self.route[min(self.leg, len(self.route) - 1)]

    def _bearing(self, obs: Observation) -> float:
        d = self.target['position'][:2] - obs.position_enu_m[:2]
        return math.atan2(d[1], d[0])

    # WaypointController hooks: where to go, which way to face
    def current_target_enu_m(self) -> np.ndarray:
        goal = self.target['position']
        if self.phase == 'takeoff':
            return np.array([self.hold_xy[0], self.hold_xy[1],
                             self.takeoff_z + self.demo['takeoff_height_m']])
        if self.phase == 'turn':  # turn on the spot, climbing to the balloon's height
            return np.array([self.hold_xy[0], self.hold_xy[1], goal[2]])
        return goal.copy()

    def _heading_target(self) -> float:
        obs = self._obs
        if obs is None or self.phase == 'takeoff':
            return self.yaw_start  # hold the take-off heading
        distance = float(np.linalg.norm(self.target['position'][:2] - obs.position_enu_m[:2]))
        if distance < self.demo['heading_freeze_distance_m']:
            return self.yaw_sp  # bearing undefined at the balloon: keep heading
        # nearest equivalent of the bearing to the current (unwrapped) heading
        return self.yaw_unwrapped + wrap_angle(self._bearing(obs) - self.yaw_unwrapped)

    # -- controller interface ----------------------------------------------

    def reset(self, obs: Observation, status: FcStatus) -> None:
        super().reset(obs, status)
        self.leg = 0
        self.phase = 'takeoff'
        self.hold_xy = obs.position_enu_m[:2].copy()
        self.takeoff_z = obs.position_enu_m[2]
        self._obs = obs
        self.log.append((obs.time_s, 'take-off'))

    def done(self) -> bool:
        return self.leg >= len(self.route)

    def update(self, obs: Observation, status: FcStatus, dt: float) -> StickCommand:
        self._obs = obs
        goal = self.target['position']
        if self.phase == 'takeoff':
            climbed = obs.position_enu_m[2] - self.takeoff_z
            if climbed > self.demo['takeoff_height_m'] - 0.1:
                self.phase = 'turn'
                self.hold_xy = obs.position_enu_m[:2].copy()
                self.log.append((obs.time_s, f"leg 1: turn towards {self.target['name']}"))
        elif self.phase == 'turn':
            heading_error = abs(wrap_angle(self._bearing(obs) - obs.yaw_rad))
            if heading_error < math.radians(self.demo['face_tolerance_deg']):
                self.phase = 'approach'
                self.log.append((obs.time_s, f"facing {self.target['name']}: approach"))
        elif np.linalg.norm(goal - obs.position_enu_m) < self.demo['pop_distance_m']:
            self.log.append((obs.time_s, f"reached {self.target['name']} "
                                         f"({self.target['points']} points)"))
            self.leg += 1
            if not self.done():
                self.phase = 'turn'
                self.hold_xy = obs.position_enu_m[:2].copy()
                self.log.append((obs.time_s, f"leg {self.leg + 1}: turn towards "
                                             f"{self.target['name']}"))
        command = super().update(obs, status, dt)
        self.index = 0  # the base class never advances: this class sequences
        return command

    def setpoint_enu_m(self):
        return None if self.done() else self.current_target_enu_m()

    def diagnostics(self) -> Mapping[str, Any]:
        base = dict(super().diagnostics())
        base.pop('wp', None)
        return {'leg': f'{min(self.leg + 1, len(self.route))}/{len(self.route)}',
                'target': self.target['name'], 'phase': self.phase, **base}
