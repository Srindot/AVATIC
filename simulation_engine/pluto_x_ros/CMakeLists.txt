cmake_minimum_required(VERSION 3.16)
project(pluto_x_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluto_x_interfaces REQUIRED)
find_package(pluto_x_core REQUIRED)  # telemetry wire layout (header only)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(fc_link src/fc_link_node.cpp)
target_link_libraries(fc_link
  rclcpp::rclcpp
  pluto_x_core::pluto_x_core
  ${pluto_x_interfaces_TARGETS}
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
target_compile_options(fc_link PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS fc_link DESTINATION lib/${PROJECT_NAME})
# MSP-over-TCP test bridge for the hardware backend (Python)
install(PROGRAMS scripts/msp_sim_bridge.py DESTINATION lib/${PROJECT_NAME})
ament_package()
