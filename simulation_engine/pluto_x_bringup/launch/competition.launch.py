"""COMPETITION / PRACTICE RUN: the arena, flown by YOUR controller.

  ros2 launch pluto_x_bringup competition.launch.py controller:=outerloop_controller/my_controller.py
      simulator + your controller, one command
  ros2 launch pluto_x_bringup competition.launch.py
      simulator only; then run your controller in a second terminal:
      python3 outerloop_controller/my_controller.py

  arena_seed:=analysis        (default) the development layout, seed in
                               analysis/seed.yaml (new one: python3 analysis/new_seed.py)
  arena_seed:=random          a fresh random layout;  arena_seed:=42  that layout
  record:=false               don't save the run (default: analysis/runs/<date-time>/)
  headless:=true rviz:=false  no windows

The simulator waits for your controller (no built-in autopilot). The 15 s
run clock starts when your controller ARMS the drone; at the end the
simulation pauses and the scoreboard prints the result.

Arguments: controller (path to your .py, '' = none), arena_seed ('analysis' |
'random' | number), record, record_dir, headless, rviz, time_limit_s, result_file, msp_bridge (true: the
simulated drone also answers MSP on tcp://127.0.0.1:9060 and serves H.264
video on tcp://127.0.0.1:9061 - for testing the hardware backend:
  python3 outerloop_controller/my_controller.py --hardware --host 127.0.0.1 \
      --video tcp://127.0.0.1:9061 ).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _repo_root():
    """The repository root: <root>/install/pluto_x_bringup is this package's prefix."""
    from ament_index_python.packages import get_package_prefix
    return os.path.dirname(os.path.dirname(get_package_prefix('pluto_x_bringup')))


def _resolve_controller(path):
    """Absolute path of the controller file, trying (in order) the path as
    given (absolute, or relative to the current directory) and relative to
    the repository root. Raises with every place tried."""
    expanded = os.path.expanduser(path)
    candidates = [os.path.abspath(expanded)]
    if not os.path.isabs(expanded):
        in_repo = os.path.join(_repo_root(), expanded)
        if in_repo not in candidates:
            candidates.append(in_repo)
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        f"controller file not found: '{path}'. Looked for:\n  " + '\n  '.join(candidates)
        + '\nGive the path from the repository root (e.g. '
          'controller:=outerloop_controller/my_controller.py) or an absolute path.')


def _check_controller(context):
    """Runs FIRST: a bad controller path stops the launch before anything starts."""
    path = LaunchConfiguration('controller').perform(context)
    if path:
        _resolve_controller(path)
    return []


def _msp_bridge(context):
    if LaunchConfiguration('msp_bridge').perform(context).lower() != 'true':
        return []
    from launch_ros.actions import Node
    return [Node(package='pluto_x_ros', executable='msp_sim_bridge.py', output='screen',
                 parameters=[{'use_sim_time': True}])]


def _controller(context):
    """Runs the participant's controller file, if one was given."""
    path = LaunchConfiguration('controller').perform(context)
    if not path:
        return []
    path = _resolve_controller(path)
    # the script connects and waits for the simulator itself (Drone()).
    # avatic_drone on the path, so a controller stored anywhere (e.g. a
    # submission folder) imports it like one inside outerloop_controller/
    pythonpath = os.pathsep.join(
        p for p in (os.path.join(_repo_root(), 'outerloop_controller'),
                    os.environ.get('PYTHONPATH', '')) if p)
    return [ExecuteProcess(
        cmd=[sys.executable, path], name='controller', output='screen',
        emulate_tty=True, additional_env={'PYTHONUNBUFFERED': '1', 'PYTHONPATH': pythonpath})]


def _seed_file():
    return os.path.join(_repo_root(), 'analysis', 'seed.yaml')


def _resolve_seed(value):
    """'analysis' -> analysis/seed.yaml (created with seed 42 if missing),
    'random' -> a fresh random seed, a number -> that number."""
    import yaml
    if value == 'analysis':
        path = _seed_file()
        if not os.path.isfile(path):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as out:
                out.write('# Balloon-layout seed for development runs (see analysis/README.md).\n'
                          '# Change it with: python3 analysis/new_seed.py\nseed: 42\n')
        with open(path, encoding='utf-8') as stream:
            return int(yaml.safe_load(stream)['seed'])
    if value == 'random':
        import random
        return random.SystemRandom().randrange(1, 1_000_000)
    return int(value)


def _new_run_dir(root, name=''):
    from datetime import datetime
    base = os.path.join(root, name or datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    path, n = base, 1
    while os.path.exists(path):
        n += 1
        path = f'{base}_{n}'
    os.makedirs(path)
    return path


def _setup(context):
    import subprocess
    import tempfile

    import yaml
    from ament_index_python.packages import get_package_prefix
    from launch_ros.actions import Node

    seed = _resolve_seed(LaunchConfiguration('arena_seed').perform(context))
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    root = LaunchConfiguration('record_dir').perform(context) or \
        os.path.join(_repo_root(), 'analysis', 'runs')
    run_name = LaunchConfiguration('run_name').perform(context)
    run_dir = _new_run_dir(os.path.abspath(root), run_name) if record else \
        tempfile.mkdtemp(prefix='pluto_run_')

    # the arena of this run, generated into the run directory
    gazebo = get_package_share_directory('pluto_x_gazebo')
    layout = os.path.join(run_dir, 'layout.yaml')
    generator = os.path.join(get_package_prefix('pluto_x_gazebo'), 'lib', 'pluto_x_gazebo',
                             'generate_arena.py')
    result = subprocess.run(
        [sys.executable, generator, '--seed', str(seed), '--base',
         os.path.join(gazebo, 'config', 'arena_default.yaml'), '--out', layout],
        capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f'arena generation failed: {result.stderr.strip()}')
    with open(layout, encoding='utf-8') as stream:
        arena = yaml.safe_load(stream)
    arena['time_limit_s'] = float(LaunchConfiguration('time_limit_s').perform(context))
    if not 0.0 < arena['time_limit_s'] < 3600.0:   # also rejects nan
        raise RuntimeError(f"time_limit_s must be in (0, 3600) s, got {arena['time_limit_s']}")
    arena['clock_start'] = 'armed'          # the 15 s start when the controller arms
    if LaunchConfiguration('msp_bridge').perform(context).lower() == 'true':
        # a real drone does not freeze at the time limit: keep the physics
        # running so the hardware backend's landing can be tested
        arena['pause_on_time_limit'] = False
    with open(layout, 'w', encoding='utf-8') as out:
        out.write(f'# arena for this run: seed {seed}\n')
        yaml.safe_dump(arena, out, sort_keys=False, default_flow_style=None)
    print(f'[competition] balloon layout seed {seed}'
          + (f'; recording to {run_dir}' if record else ' (not recorded)'))

    controller = LaunchConfiguration('controller').perform(context)
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pluto_x_bringup'), 'launch', 'arena.launch.py')),
        launch_arguments={
            'outer_loop': 'false',              # the participant's script flies
            'arena_config': layout,
            # launch arguments are shared with included files: blank the ones
            # this file has already applied to `layout`
            'arena_seed': '', 'time_limit_s': '', 'clock_start': '',
            'headless': LaunchConfiguration('headless').perform(context),
            'rviz': LaunchConfiguration('rviz').perform(context),
            'result_file': LaunchConfiguration('result_file').perform(context) or
            os.path.join(run_dir, 'result_arena.yaml'),
        }.items())]
    if record:
        actions.append(Node(
            package='pluto_x_bringup', executable='run_recorder.py', name='run_recorder',
            output='screen', parameters=[{
                'use_sim_time': True, 'run_dir': run_dir, 'seed': str(seed),
                'controller': _resolve_controller(controller) if controller else '',
                'arena_config': layout,
                'vehicle_config': os.path.join(get_package_share_directory('pluto_x_core'),
                                               'config', 'pluto_x_estimated.yaml')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='',
                              description="your controller .py ('' = run it yourself)"),
        DeclareLaunchArgument('arena_seed', default_value='analysis',
                              description="'analysis' (analysis/seed.yaml), 'random' or a number"),
        DeclareLaunchArgument('record', default_value='true',
                              description='save this run to analysis/runs/<date-time>/'),
        DeclareLaunchArgument('record_dir', default_value='',
                              description="where runs are saved ('' = analysis/runs)"),
        DeclareLaunchArgument('run_name', default_value='',
                              description="run directory name ('' = the date and time)"),
        DeclareLaunchArgument('msp_bridge', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('time_limit_s', default_value='15'),
        DeclareLaunchArgument('result_file', default_value=''),
        OpaqueFunction(function=_check_controller),
        OpaqueFunction(function=_setup),
        OpaqueFunction(function=_msp_bridge),
        OpaqueFunction(function=_controller),
    ])
