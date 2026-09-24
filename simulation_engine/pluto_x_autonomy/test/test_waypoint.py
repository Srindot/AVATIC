import math

import numpy as np
import pytest

from pluto_x_autonomy.examples.waypoint import WaypointController
from sim_helpers import PointMass


def command_towards(target, yaw_deg):
    vehicle = PointMass()
    vehicle.yaw = math.radians(yaw_deg)
    controller = WaypointController({'waypoints_enu_m': [target]})
    controller.reset(vehicle.observation(0.0), vehicle.status(0.0))
    return controller.update(vehicle.observation(0.02), vehicle.status(0.02),
                             0.02)


def test_signs_facing_north():
    ahead = command_towards([0, 2, 0], yaw_deg=90)   # target north
    assert ahead.pitch > 0.1 and abs(ahead.roll) < 1e-9
    right = command_towards([2, 0, 0], yaw_deg=90)   # target east = right
    assert right.roll > 0.1 and abs(right.pitch) < 1e-9
    up = command_towards([0, 0, 1], yaw_deg=90)
    assert up.throttle > 0.72


def test_signs_facing_east():
    ahead = command_towards([2, 0, 0], yaw_deg=0)
    assert ahead.pitch > 0.1 and abs(ahead.roll) < 1e-9
    left = command_towards([0, 2, 0], yaw_deg=0)     # north = left
    assert left.roll < -0.1


def test_tilt_limited():
    far = command_towards([100, 0, 0], yaw_deg=0)
    assert far.pitch * 32.0 <= 15.0 + 1e-6


def test_rejects_bad_params():
    with pytest.raises(ValueError):
        WaypointController({'waypoints_enu_m': []})
    with pytest.raises(ValueError):
        WaypointController({'typo_gain': 1.0})
