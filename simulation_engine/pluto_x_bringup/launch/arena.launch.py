"""Launch the AVATIC balloon arena: sim.launch.py in the arena world, plus
the camera and arena bridges and RViz.

The world (balloon_arena.sdf.xacro) is generated at launch time with the
arena configuration: balloons (coloured, scaled), popping on contact, score,
and a run time limit after which the simulation pauses.

Arguments (plus every sim.launch.py argument except `world`):
  arena_config   arena YAML (default pluto_x_gazebo config/arena_default.yaml)
  result_file    write the final result YAML here ('' = do not write)
  time_limit_s   override the arena's run time limit ('' = from arena_config)
  clock_start    override when the run clock starts: sim_start | armed
  arena_seed     'random', or a number: random balloon layout (generate_arena.py:
                 spacing >= drone width, compact cluster); '' = the
                 layout in arena_config
  rviz           true: open RViz with the camera view (default true)

ROS topics:
  /pluto/camera/image_raw, /pluto/camera/camera_info   vehicle camera
  /arena/score (std_msgs/Int32), /arena/time_remaining (std_msgs/Float64),
  /arena/events, /arena/result (std_msgs/String)
  /arena/viz (visualization_msgs/MarkerArray): score text + flight path,
  from arena_scoreboard, which also prints the result table at the end

Examples:
  ros2 launch pluto_x_bringup arena.launch.py outer_loop:=false
  ros2 launch pluto_x_bringup arena.launch.py controller:=/path/mine.py:Mine
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PASS_THROUGH = ['flight_controller', 'config_file', 'headless', 'paused',
                'spawn_yaw', 'outer_loop', 'controller',
                'controller_params_file', 'shutdown_after_mission']


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def _launch_setup(context):
    arena_config = os.path.abspath(
        LaunchConfiguration('arena_config').perform(context))
    if not os.path.isfile(arena_config):
        raise RuntimeError(f'arena_config not found: {arena_config}')
    seed = LaunchConfiguration('arena_seed').perform(context)
    if seed == 'random':
        import random
        seed = str(random.SystemRandom().randrange(1, 1_000_000))
        print(f'[arena] random layout: arena_seed:={seed} (use it to replay this layout)')
    if seed:
        # random layout from the seed (rules from arena_config)
        import subprocess
        import sys
        generated = os.path.join(tempfile.mkdtemp(prefix='pluto_arena_gen_'), 'arena.yaml')
        generator = os.path.join(get_package_prefix('pluto_x_gazebo'), 'lib',
                                 'pluto_x_gazebo', 'generate_arena.py')
        result = subprocess.run([sys.executable, generator, '--seed', seed,
                                 '--base', arena_config, '--out', generated],
                                capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'arena generation failed: {result.stderr.strip()}')
        arena_config = generated
    time_limit = LaunchConfiguration('time_limit_s').perform(context)
    clock_start = LaunchConfiguration('clock_start').perform(context)
    if clock_start and clock_start not in ('sim_start', 'armed'):
        raise RuntimeError("clock_start must be 'sim_start' or 'armed'")
    if time_limit or clock_start:
        # copy of the arena config with the time limit replaced (the file on
        # disk is not changed)
        import yaml
        with open(arena_config, 'r', encoding='utf-8') as stream:
            arena = yaml.safe_load(stream)
        if time_limit:
            arena['time_limit_s'] = float(time_limit)
            if not arena['time_limit_s'] > 0.0:
                raise RuntimeError('time_limit_s must be > 0')
        if clock_start:
            arena['clock_start'] = clock_start
        arena_config = os.path.join(tempfile.mkdtemp(prefix='pluto_arena_cfg_'),
                                    'arena.yaml')
        with open(arena_config, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(arena, stream, sort_keys=False)
    result_file = LaunchConfiguration('result_file').perform(context)
    mappings = {
        'arena_config': arena_config,
        'profile_file': _share('pluto_x_gazebo', 'models', 'balloon', 'profile.yaml'),
    }
    if result_file:
        mappings['result_file'] = os.path.abspath(result_file)
    world_sdf = xacro.process_file(
        _share('pluto_x_gazebo', 'worlds', 'balloon_arena.sdf.xacro'),
        mappings=mappings).toxml()
    world_file = os.path.join(tempfile.mkdtemp(prefix='pluto_arena_'),
                              'balloon_arena.sdf')
    with open(world_file, 'w', encoding='utf-8') as stream:
        stream.write(world_sdf)

    arguments = {name: LaunchConfiguration(name).perform(context)
                 for name in PASS_THROUGH}
    arguments['world'] = world_file
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('pluto_x_bringup', 'launch', 'sim.launch.py')),
            launch_arguments=arguments.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='arena_bridge', output='screen',
             parameters=[{'config_file': _share('pluto_x_bringup', 'config',
                                                'arena_bridge.yaml'),
                          'use_sim_time': True}]),
    ]
    actions.append(Node(
        package='pluto_x_bringup', executable='arena_scoreboard.py',
        name='arena_scoreboard', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': True}]))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', _share('pluto_x_bringup', 'rviz', 'arena.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arena_config',
            default_value=_share('pluto_x_gazebo', 'config', 'arena_default.yaml')),
        DeclareLaunchArgument('result_file', default_value=''),
        DeclareLaunchArgument('clock_start', default_value='',
                              description="override: sim_start | armed ('' = keep)"),
        DeclareLaunchArgument('arena_seed', default_value='',
                              description="random balloon layout from this seed ('' = arena_config as is)"),
        DeclareLaunchArgument('time_limit_s', default_value='',
                              description="override the arena's time_limit_s ('' = keep)"),
        DeclareLaunchArgument('rviz', default_value='true'),
        # sim.launch.py arguments (defaults as there)
        DeclareLaunchArgument('flight_controller', default_value=''),
        DeclareLaunchArgument(
            'config_file',
            default_value=_share('pluto_x_core', 'config', 'pluto_x_estimated.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('outer_loop', default_value='true'),
        DeclareLaunchArgument(
            'controller',
            default_value='pluto_x_autonomy.examples.waypoint:WaypointController'),
        DeclareLaunchArgument(
            'controller_params_file',
            default_value=_share('pluto_x_autonomy', 'config', 'arena_demo.yaml')),
        DeclareLaunchArgument('shutdown_after_mission', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
