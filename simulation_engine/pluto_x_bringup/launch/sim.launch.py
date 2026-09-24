"""Launch the Pluto X simulation (ROS 2 Humble + Gazebo Harmonic).

Starts:
  * Gazebo Sim with the given world (GUI optional)
  * the Pluto X model (xacro from the vehicle YAML) with the VehicleSystem
    plugin, flown by the selected flight controller
  * ros_gz_bridge (clock, IMU, battery, ground-truth odometry)
  * fc_link: RC (ROS /pluto/rc -> sim) and flight-controller telemetry
    (sim -> ROS /pluto/fc_status), the stand-in for the Wi-Fi/MSP link
  * outer_loop_host (if outer_loop:=true) running the given controller

Arguments:
  flight_controller       magisv2 | legacy | '' (default '': from the YAML;
                          pluto_x_estimated.yaml selects magisv2)
  config_file             vehicle YAML (default pluto_x_core
                          pluto_x_estimated.yaml)
  world                   world SDF (default pluto_x_gazebo legacy_flat.sdf)
  headless                true: server only, no GUI
  paused                  true: start paused
  spawn_yaw               initial ENU yaw [rad] (0 faces +x, east)
  outer_loop              true: start outer_loop_host (default true)
  controller              'package.module:Class' or '/file.py:Class'
                          (default: the example WaypointController)
  controller_params_file  YAML passed to the controller (default: the
                          example square mission)

Examples:
  ros2 launch pluto_x_bringup sim.launch.py
  ros2 launch pluto_x_bringup sim.launch.py \\
      controller:=/home/me/my_ctrl.py:MyController \\
      controller_params_file:=/home/me/my_params.yaml
  ros2 launch pluto_x_bringup sim.launch.py outer_loop:=false   # drive
      /pluto/rc yourself
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _launch_setup(context):
    config_file = os.path.abspath(
        LaunchConfiguration('config_file').perform(context))
    flight_controller = LaunchConfiguration('flight_controller').perform(context)
    world = LaunchConfiguration('world').perform(context)
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))
    headless = _flag(context, 'headless')
    paused = _flag(context, 'paused')

    if not os.path.isfile(config_file):
        raise RuntimeError(f'config_file not found: {config_file}')
    with open(config_file, 'r', encoding='utf-8') as stream:
        spawn_height_m = float(
            yaml.safe_load(stream)['gazebo_model']['spawn_height_m'])

    model_sdf = xacro.process_file(
        _share('pluto_x_gazebo', 'models', 'pluto_x', 'model.sdf.xacro'),
        mappings={'config_file': config_file,
                  'flight_controller': flight_controller}).toxml()

    gz_args = (f'{"" if paused else "-r "}'
               f'{"-s --headless-rendering " if headless else ""}'
               f'-v 3 {world}')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share('ros_gz_sim', 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args,
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-string', model_sdf, '-name', 'pluto_x',
                        '-x', '0', '-y', '0', '-z', str(spawn_height_m),
                        '-Y', str(spawn_yaw)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             output='screen',
             parameters=[{'config_file': _share('pluto_x_bringup', 'config',
                                                'bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='pluto_x_ros', executable='fc_link', output='screen',
             parameters=[{'use_sim_time': True}]),
    ]
    if _flag(context, 'outer_loop'):
        host = Node(
            package='pluto_x_autonomy', executable='outer_loop_host',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'controller': LaunchConfiguration('controller').perform(context),
                'controller_params_file': LaunchConfiguration(
                    'controller_params_file').perform(context),
            }])
        actions.append(host)
        if _flag(context, 'shutdown_after_mission'):
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=host,
                on_exit=[EmitEvent(event=Shutdown(reason='mission finished'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('flight_controller', default_value=''),
        DeclareLaunchArgument(
            'config_file',
            default_value=_share('pluto_x_core', 'config',
                                 'pluto_x_estimated.yaml')),
        DeclareLaunchArgument(
            'world',
            default_value=_share('pluto_x_gazebo', 'worlds', 'legacy_flat.sdf')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('outer_loop', default_value='true'),
        DeclareLaunchArgument(
            'controller',
            default_value='pluto_x_autonomy.examples.waypoint:WaypointController'),
        DeclareLaunchArgument(
            'controller_params_file',
            default_value=_share('pluto_x_autonomy', 'config',
                                 'waypoint_square.yaml')),
        DeclareLaunchArgument('shutdown_after_mission', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
