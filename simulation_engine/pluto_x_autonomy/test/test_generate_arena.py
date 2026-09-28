"""Property test of pluto_x_gazebo/tools/generate_arena.py (the random arena)."""
import itertools
import math
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(__file__)
TOOLS = os.path.join(HERE, '..', '..', 'pluto_x_gazebo', 'tools')
sys.path.insert(0, TOOLS)
import generate_arena as gen  # noqa: E402

PARAMS = {'diameter_m': 0.3, 'drone_width_m': 0.16, 'max_radius_m': 3.5,
          'max_neighbour_distance_m': 2.0, 'min_takeoff_distance_m': 1.0,
          'min_height_m': 0.8, 'max_height_m': 2.0, 'boundary_margin_m': 0.5}


def run(*args):
    out = subprocess.run([sys.executable, os.path.join(TOOLS, 'generate_arena.py'), *args],
                         capture_output=True, text=True, check=True)
    return yaml.safe_load(out.stdout)


def test_layouts_satisfy_constraints():
    for seed in range(40):
        arena = run('--seed', str(seed))
        positions = [b['position_enu_m'] for b in arena['balloons']]
        assert gen.check_layout(positions, PARAMS) == [], seed
        closest = min(gen.horizontal(a, b) for a, b in itertools.combinations(positions, 2))
        assert closest >= 0.30 + 0.16 - 1e-6  # surfaces at least a drone width apart
        assert max(math.hypot(p[0], p[1]) for p in positions) <= 3.5 + 1e-6
        assert sorted(b['color'] for b in arena['balloons']) == \
            sorted(['yellow', 'yellow', 'blue', 'blue', 'red', 'red', 'green', 'green'])
        assert arena['colors']['red']['points'] == -300  # rules come from the base


def test_reproducible_and_seed_dependent():
    assert run('--seed', '5') == run('--seed', '5')
    assert run('--seed', '5')['balloons'] != run('--seed', '6')['balloons']


def test_checker_catches_violations():
    too_close = [[1.5, 0.0, 1.0], [1.5, 0.3, 1.0]]
    assert any('centre distance' in p for p in gen.check_layout(too_close, PARAMS))
    scattered = [[1.5, 0.0, 1.0], [-3.0, 0.0, 1.0]]
    assert any('cluster' in p for p in gen.check_layout(scattered, PARAMS))
