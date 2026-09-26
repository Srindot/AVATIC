"""WORKSHOP DEMO: arena run popping balloons along a hand-planned route.

  ros2 launch pluto_x_demo balloon_demo.launch.py

Opens Gazebo (the whole environment) and RViz (the drone's camera, the live
score and the flight path). The demo controller flies to KNOWN balloon
positions using ground truth and turns to face each balloon before and
while approaching it (demo/pluto_x_demo/balloon_demo.py,
demo/config/balloon_demo.yaml). Teams have neither: their controller must find
the balloons in the camera image.

The demo runs for 35 s (time_limit_s; the competition limit is 25 s), long
enough to visit the six non-red balloons. Then the simulation pauses and the
scoreboard prints the result table in this terminal (and shows the final
score in RViz). time_limit_s:=25 shows a run of the competition length.

Arguments: route_file (default pluto_x_demo config/balloon_demo.yaml),
time_limit_s (default 35), headless, rviz, result_file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('pluto_x_bringup')
    demo = get_package_share_directory('pluto_x_demo')
    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file', default_value=os.path.join(demo, 'config', 'balloon_demo.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('result_file', default_value=''),
        # longer than the 25 s competition run, so the demo can visit
        # every balloon
        DeclareLaunchArgument('time_limit_s', default_value='35'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'arena.launch.py')),
            launch_arguments={
                'controller': 'pluto_x_demo.balloon_demo:BalloonDemoController',
                'controller_params_file': LaunchConfiguration('route_file'),
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'result_file': LaunchConfiguration('result_file'),
                'time_limit_s': LaunchConfiguration('time_limit_s'),
                # the route is for the default layout: never a seeded one
                'arena_seed': '',
            }.items()),
    ])
