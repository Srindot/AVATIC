"""The camera-visibility geometry of runlog (plot_visibility).

    python3 -m pytest -q analysis/tests
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import runlog  # noqa: E402


def _run(yaw_rad, balloons, events=None):
    n = 21
    t = np.linspace(0.0, 2.0, n)
    zeros = np.zeros(n)
    trajectory = {'t_s': t, 'x_enu_m': zeros, 'y_enu_m': zeros, 'z_enu_m': zeros + 1.0,
                  'roll_rad': zeros, 'pitch_rad': zeros, 'yaw_rad': zeros + yaw_rad}
    return runlog.Run(run_id='t', path='.', meta={}, result=None, trajectory=trajectory,
                      balloons=[{'name': name, 'colour': 'green', 'points': 100,
                                 'position': np.array(pos, dtype=float), 'popped': False}
                                for name, pos in balloons],
                      telemetry={}, commands={}, events=events or {}, camera_frames={})


def test_ahead_is_seen_behind_and_outside_the_view_are_not():
    run = _run(0.0, [('ahead', (2.0, 0.0, 1.0)), ('behind', (-2.0, 0.0, 1.0)),
                     ('left_60deg', (1.0, 1.732, 1.0)), ('left_30deg', (1.732, 1.0, 1.0))])
    vis = runlog.balloon_visibility(run)
    assert np.isfinite(vis['ahead']).all() and np.isfinite(vis['left_30deg']).all()
    assert np.isnan(vis['behind']).all() and np.isnan(vis['left_60deg']).all()
    # 0.30 m at 2 m with an 80 deg, 1280 px camera: 640 / tan(40 deg) * 0.15 = about 114 px
    assert abs(np.nanmean(vis['ahead']) - 0.30 * 640 / np.tan(np.radians(40)) / 2.0) < 3


def test_heading_turns_the_view_and_a_pop_removes_the_balloon():
    # yaw 90 deg (REP-103, CCW from east) faces north
    events = {'t_s': np.array([1.0]), 'kind': np.array(['pop']),
              'text': np.array(['t=1.0 s: POP green +100 (north, run time 1 s) - total 100'])}
    vis = runlog.balloon_visibility(_run(np.pi / 2, [('north', (0.0, 2.0, 1.0)),
                                                     ('east', (2.0, 0.0, 1.0))], events))
    assert np.isnan(vis['east']).all()
    seen = np.isfinite(vis['north'])
    assert seen[vis['t_s'] < 1.0].all() and not seen[vis['t_s'] >= 1.0].any()
