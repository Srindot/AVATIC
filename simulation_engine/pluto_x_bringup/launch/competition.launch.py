"""COMPETITION / PRACTICE RUN: the arena, flown by YOUR controller.

  ros2 launch pluto_x_bringup competition.launch.py              # new random layout
  ros2 launch pluto_x_bringup competition.launch.py arena_seed:=42    # replay a layout
  ros2 launch pluto_x_bringup competition.launch.py headless:=true rviz:=false

then, in a second terminal:
  python3 outerloop_controller/my_controller.py

The simulator waits for your controller (no built-in autopilot). The 15 s
run clock starts when your controller ARMS the drone; at the end the
simulation pauses and the scoreboard prints the result.

Arguments: arena_seed ('random' | number), headless, rviz, time_limit_s,
result_file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('pluto_x_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('arena_seed', default_value='random'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('time_limit_s', default_value='15'),
        DeclareLaunchArgument('result_file', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'arena.launch.py')),
            launch_arguments={
                'outer_loop': 'false',          # the participant's script flies
                'clock_start': 'armed',         # the 15 s start when they arm
                'arena_seed': LaunchConfiguration('arena_seed'),
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'time_limit_s': LaunchConfiguration('time_limit_s'),
                'result_file': LaunchConfiguration('result_file'),
            }.items()),
    ])
