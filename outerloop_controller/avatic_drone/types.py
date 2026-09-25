"""Data types shared by the simulator and hardware backends (no ROS here)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass(frozen=True)
class Frame:
    """One camera image. image: H x W x 3 uint8, RGB.

    seq counts frames from 1; the camera (~18 Hz) is slower than a typical
    control loop, so compare seq to skip frames you have already processed.
    """
    image: np.ndarray
    time_s: float
    width: int
    height: int
    seq: int


@dataclass(frozen=True)
class Command:
    """What your controller sends each step (see the module docstring).

    roll, pitch in -1..1 are ANGLE setpoints (+roll = bank right, +pitch =
    nose down / forward); yaw_rate in -1..1 is a turn RATE (+ = clockwise);
    throttle in 0..1 (hover ~0.76). altitude_hold engages the firmware's
    barometric altitude hold (then throttle ~0.5 = hold, above = climb).
    WARNING: in the simulator, engaging altitude hold in flight makes the
    drone drop (the firmware assumes hover at throttle 0.5, the simulated
    drone needs ~0.76; docs/architecture.md firmware finding 7). Fly with
    your own altitude loop (altitude_hold=False) until this is resolved.
    """
    roll: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0
    altitude_hold: bool = False


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


@dataclass(frozen=True)
class SafetyLimits:
    """Caps applied to every command by BOTH backends (simulator and real
    drone), whatever the controller asks for. Commands beyond them are
    clipped (a warning is printed once).

      max_tilt_stick      |roll|, |pitch| <= 0.6  (the firmware's angle already
                          saturates at 20 deg from ~0.45 stick)
      max_yaw_rate_stick  |yaw_rate| <= 0.8        (~62 deg/s)
      max_throttle        throttle <= 0.95
      max_altitude_m      above this barometric altitude the throttle is
                          limited to below hover, so the drone descends
    """
    max_tilt_stick: float = 0.6
    max_yaw_rate_stick: float = 0.8
    max_throttle: float = 0.95
    max_altitude_m: float = 2.5


SAFETY_LIMITS = SafetyLimits()


def apply_safety_limits(command: 'Command', limits: SafetyLimits = SAFETY_LIMITS):
    """(capped Command, list of the caps that were active)."""
    capped = []

    def cap(name, value, limit, low=None):
        low = -limit if low is None else low
        if value > limit or value < low:
            capped.append(name)
        return min(max(value, low), limit)

    return Command(roll=cap('tilt', command.roll, limits.max_tilt_stick),
                   pitch=cap('tilt', command.pitch, limits.max_tilt_stick),
                   yaw_rate=cap('yaw rate', command.yaw_rate, limits.max_yaw_rate_stick),
                   throttle=cap('throttle', command.throttle, limits.max_throttle, low=0.0),
                   altitude_hold=command.altitude_hold), sorted(set(capped))
