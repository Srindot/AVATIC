"""Fair-play checks: the code check (check_controller.py) and the running
controller's monitor (pluto_x_bringup/scripts/integrity_monitor.py).

    python3 -m pytest -q evaluation/tests
"""

import importlib.util
import os
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
import check_controller  # noqa: E402

_spec = importlib.util.spec_from_file_location('integrity_monitor', os.path.join(
    REPO, 'simulation_engine', 'pluto_x_bringup', 'scripts', 'integrity_monitor.py'))
monitor = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(monitor)


def _check(tmp_path, source, extra=None):
    path = tmp_path / 'controller.py'
    path.write_text(textwrap.dedent(source))
    for name, text in (extra or {}).items():
        (tmp_path / name).write_text(textwrap.dedent(text))
    return check_controller.check_controller(str(path))


# ------------------------------------------------------------- code check

def test_shipped_controllers_are_clean():
    for path in ('outerloop_controller/my_controller.py',
                 'outerloop_controller/examples/hello_drone.py'):
        assert check_controller.check_controller(os.path.join(REPO, path)) == [], path


def test_ordinary_vision_code_is_clean(tmp_path):
    assert _check(tmp_path, '''
        import math
        import cv2
        import numpy as np
        from avatic_drone import Command
        HSV = {'green': ((40, 80, 60), (80, 255, 255))}
        def mask(image):
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            return cv2.inRange(hsv, np.array(HSV['green'][0]), np.array(HSV['green'][1]))
        class MyController:
            def __init__(self):
                self._last = None
            def step(self, frame, telemetry, t):
                self._last = math.radians(telemetry.heading_deg)
                return Command(throttle=0.76)
        ''') == []


def test_ground_truth_access_is_found(tmp_path):
    findings = _check(tmp_path, '''
        import os
        import rclpy
        from nav_msgs.msg import Odometry
        runs = os.listdir('analysis/runs')
        data = open('layout.yaml').read()
        mod = __import__('sub' + 'process')
        def cheat(drone):
            drone._link.create_subscription(Odometry, '/sim/pluto/odometry', print, 1)
        ''')
    text = '\n'.join(str(f) for f in findings)
    for expected in ('import rclpy', 'import nav_msgs.msg', 'os.listdir', 'open()',
                     '__import__()', '._link', '/sim/', 'layout'):
        assert expected in text, expected
    assert check_controller.verdict(findings) == 'review'


def test_local_modules_are_checked_too(tmp_path):
    findings = _check(tmp_path, 'import helpers\n', {'helpers.py': 'import gz.transport13\n'})
    assert [(f.path, f.kind) for f in findings] == [('helpers.py', 'forbidden')]


def test_environment_only_for_avatic_settings(tmp_path):
    findings = _check(tmp_path, '''
        import os
        backend = os.environ.get('AVATIC_BACKEND', 'sim')
        partition = os.environ.get('GZ_PARTITION')
        ''')
    assert [f.line for f in findings if 'environment' in f.text] == [4]


def test_docstrings_may_mention_anything(tmp_path):
    assert _check(tmp_path, '''
        """Never reads /sim/pluto/odometry or the layout: camera only."""
        ''') == []


# ---------------------------------------------------------- the ROS graph

API = sorted(monitor.API_SUBSCRIPTIONS)
ORGANISER = {'/arena_scoreboard', '/run_recorder', '/fc_link', '/transform_listener_impl_*'}


def _graph(nodes, subs=None, pubs=None, clients=None):
    return monitor.check_graph(nodes, subs or {}, pubs or {}, clients or {}, ORGANISER)


def test_a_normal_run_is_clean():
    flags, notes = _graph(
        ['/arena_scoreboard', '/run_recorder', '/fc_link', '/avatic_drone',
         '/transform_listener_impl_5a1b', '/_ros2cli_daemon_0_ab'],
        subs={'/avatic_drone': set(API), '/run_recorder': {'/sim/pluto/odometry'},
              '/arena_scoreboard': {'/sim/pluto/odometry'}},
        pubs={'/avatic_drone': {'/pluto/rc', '/rosout'}})
    assert flags == [] and notes == []


def test_controller_subscribing_to_ground_truth_is_flagged():
    flags, _ = _graph(['/avatic_drone'],
                      subs={'/avatic_drone': set(API) | {'/sim/pluto/odometry'}})
    assert flags == ['the controller subscribes to /sim/pluto/odometry '
                     '(not part of the avatic_drone API)']


def test_a_second_node_is_flagged_and_noted():
    flags, notes = _graph(['/avatic_drone', '/helper'],
                          subs={'/helper': {'/arena/viz'}}, pubs={'/helper': {'/pluto/rc'}})
    assert flags == ['another ROS program (/helper) subscribes to /arena/viz',
                     'another ROS program (/helper) publishes /pluto/rc']
    assert len(notes) == 1


def test_a_disguised_node_is_flagged():
    flags, _ = _graph(['/run_recorder', '/run_recorder'])
    assert flags == ['two ROS nodes are named /run_recorder: one of them is disguised']


def test_an_unrelated_ros_tool_is_only_a_note():
    flags, notes = _graph(['/avatic_drone', '/my_echo'], subs={'/my_echo': {'/pluto/fc_status'}})
    assert flags == [] and notes == ['another ROS program was running: /my_echo '
                                     '(judged runs have none)']


# ------------------------------------------------- the controller process

def _proc(tmp_path, pid, ppid, token=None, maps='', fds=()):
    d = tmp_path / str(pid)
    (d / 'fd').mkdir(parents=True)
    env = b'PATH=/usr/bin\0' + (f'AVATIC_RUN_TOKEN={token}\0'.encode() if token else b'')
    (d / 'environ').write_bytes(env)
    (d / 'stat').write_text(f'{pid} (python3) S {ppid} 1 1 0')
    (d / 'cmdline').write_bytes(b'python3\0controller.py\0')
    (d / 'maps').write_text(maps)
    for i, target in enumerate(fds):
        os.symlink(target, d / 'fd' / str(i))


def test_processes_are_found_by_token(tmp_path):
    _proc(tmp_path, 100, 1, token='abc')
    _proc(tmp_path, 101, 100, token='abc')        # started by the controller
    _proc(tmp_path, 102, 1, token='other')
    _proc(tmp_path, 103, 1)
    pids = monitor.controller_processes('abc', proc=str(tmp_path))
    assert pids == [100, 101]
    assert monitor.started_programs(pids, proc=str(tmp_path)) == [101]


def test_gazebo_libraries_are_flagged(tmp_path):
    _proc(tmp_path, 100, 1, maps='7f00 r-xp /usr/lib/librcl.so\n'
                                 '7f01 r-xp /usr/lib/x86_64-linux-gnu/libgz-transport13.so.13.4\n')
    _proc(tmp_path, 101, 1, maps='7f00 r-xp /usr/lib/librcl.so\n')
    assert 'libgz-transport13.so' in monitor.check_libraries(100, proc=str(tmp_path))[0]
    assert monitor.check_libraries(101, proc=str(tmp_path)) == []


def test_run_files_open_are_flagged(tmp_path):
    _proc(tmp_path, 100, 1, fds=['/home/u/avatic/analysis/runs/r1/layout.yaml',
                                 '/tmp/pluto_arena_x/balloon_arena.sdf', '/proc/200/environ',
                                 '/home/u/.ros/log/python3_100.log', '/dev/null',
                                 '/home/u/team/params.json'])
    flags = monitor.check_open_files(100, proc=str(tmp_path))
    assert sorted(f.split()[-1] for f in flags) == ['/home/u/avatic/analysis/runs/r1/layout.yaml',
                                                    '/proc/200/environ',
                                                    '/tmp/pluto_arena_x/balloon_arena.sdf']


def test_findings_are_kept_once_with_a_verdict():
    findings = monitor.Findings(['ros graph'])
    assert findings.report()['verdict'] == 'clean'
    assert findings.add(1.0, flags=['x']) == [('flag', 'x')]
    assert findings.add(2.0, flags=['x']) == []
    report = findings.report()
    assert report['verdict'] == 'flagged' and report['flags'] == [{'t_s': 1.0, 'text': 'x'}]


def test_builtins_under_another_name_are_found(tmp_path):
    findings = _check(tmp_path, '''
        imp = __import__
        r = open
        data = r('/x/an' + 'alysis/ru' + 'ns/lay' + 'out.yaml').read()
        ''')
    assert [f.kind for f in findings] == ['forbidden', 'review']


def test_names_imported_from_modules_are_checked(tmp_path):
    findings = _check(tmp_path, '''
        from avatic_drone.sim import rclpy
        from numpy import load as L
        from avatic_drone import Command, Drone
        from avatic_drone.types import SAFETY_LIMITS
        import avatic_drone
        node = avatic_drone.sim
        ''')
    assert sorted(set(f.line for f in findings)) == [2, 3, 7]
    assert any(f.kind == 'forbidden' and 'rclpy' in f.text for f in findings)


def test_an_own_copy_of_avatic_drone_is_forbidden(tmp_path):
    (tmp_path / 'avatic_drone').mkdir()
    (tmp_path / 'avatic_drone' / '__init__.py').write_text('import rclpy\n')
    findings = _check(tmp_path, 'import avatic_drone\n')
    assert [f.kind for f in findings] == ['forbidden'] and 'copy' in findings[0].text


def test_system_proc_files_are_not_flagged(tmp_path):
    _proc(tmp_path, 100, 1, fds=['/proc/cpuinfo', '/proc/100/status', '/proc/sys/kernel/x'])
    assert monitor.check_open_files(100, proc=str(tmp_path)) == []
