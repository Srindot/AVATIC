import textwrap

import pytest

from pluto_x_autonomy.examples.waypoint import WaypointController
from pluto_x_autonomy.loader import load_controller_class, make_controller


def test_module_spec():
    cls = load_controller_class(
        'pluto_x_autonomy.examples.waypoint:WaypointController')
    assert cls is WaypointController


def test_file_spec_and_params(tmp_path):
    source = tmp_path / 'mine.py'
    source.write_text(textwrap.dedent('''
        from pluto_x_autonomy.api import OuterLoopController, StickCommand

        class Mine(OuterLoopController):
            def update(self, obs, status, dt):
                return StickCommand(throttle=self.params['t'])

        class NotAController:
            pass
    '''))
    params = tmp_path / 'p.yaml'
    params.write_text('t: 0.5\n')
    controller = make_controller(f'{source}:Mine', str(params))
    assert controller.update(None, None, 0.02).throttle == 0.5
    with pytest.raises(ValueError, match='not a subclass'):
        load_controller_class(f'{source}:NotAController')
    with pytest.raises(ValueError, match='not found'):
        load_controller_class(f'{source}:Missing')
    with pytest.raises(ValueError):
        load_controller_class('no_colon_here')
