import math
import os

import numpy as np
import pytest
import yaml

import sys

HERE = os.path.dirname(__file__)
ENGINE = os.path.join(HERE, '..', '..', 'simulation_engine')
sys.path.insert(0, os.path.join(ENGINE, 'pluto_x_autonomy'))          # engine python package
sys.path.insert(0, os.path.join(ENGINE, 'pluto_x_autonomy', 'test'))  # point-mass test helper
sys.path.insert(0, os.path.join(HERE, '..'))

from pluto_x_autonomy.supervisor import Phase, Supervisor  # noqa: E402
from pluto_x_demo.balloon_demo import BalloonDemoController  # noqa: E402
from sim_helpers import PointMass  # noqa: E402

ARENA = os.path.join(ENGINE, 'pluto_x_gazebo', 'config', 'arena_default.yaml')


def demo_params(**overrides):
    with open(os.path.join(HERE, '..', 'config', 'balloon_demo.yaml')) as stream:
        params = yaml.safe_load(stream)
    params['arena_config'] = ARENA
    params.update(overrides)
    return params


def test_route_is_flown_in_order_facing_each_balloon():
    controller = BalloonDemoController(demo_params())
    supervisor = Supervisor(controller)
    vehicle = PointMass()
    vehicle.yaw = 0.0  # facing east, as in the arena
    t, worst_heading_error = 0.0, 0.0
    for _ in range(int(120 / 0.02)):
        t += 0.02
        command, arm = supervisor.step(t, vehicle.observation(t), vehicle.status(t))
        vehicle.step(command, arm, 0.02)
        if controller.phase == 'approach' and not controller.done():
            to_balloon = controller.target['position'][:2] - vehicle.p[:2]
            if np.linalg.norm(to_balloon) > 0.4:
                error = abs((math.atan2(to_balloon[1], to_balloon[0]) - vehicle.yaw
                             + math.pi) % (2 * math.pi) - math.pi)
                worst_heading_error = max(worst_heading_error, math.degrees(error))
        if supervisor.finished:
            break
    reached = [text for _, text in controller.log if text.startswith('reached')]
    assert [r.split()[1] for r in reached] == [b['name'] for b in controller.route]
    assert len(reached) == len(demo_params()['route'])
    assert all(b['points'] > 0 for b in controller.route)  # never targets a red (penalty)
    assert worst_heading_error < 30.0
    assert supervisor.phase == Phase.DISARMED


def test_bad_route_rejected():
    with pytest.raises(ValueError):
        BalloonDemoController(demo_params(route=[99]))
    with pytest.raises(ValueError):
        BalloonDemoController(demo_params(route=[]))
