cmake_minimum_required(VERSION 3.16)
project(pluto_x_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/compare_trajectories.py
  scripts/dynamics_check.py
  scripts/legacy_flight_check.py
  scripts/mission_check.py
  scripts/yaw_check.py
  scripts/arena_check.py
  scripts/arena_scoreboard.py
  scripts/run_recorder.py
  scripts/integrity_monitor.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
