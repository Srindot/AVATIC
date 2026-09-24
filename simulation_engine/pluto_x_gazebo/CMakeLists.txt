cmake_minimum_required(VERSION 3.16)
project(pluto_x_gazebo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluto_x_core REQUIRED)
find_package(pluto_x_magisv2 REQUIRED)
# Gazebo Harmonic libraries (installed with ros-humble-ros-gzharmonic).
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(yaml-cpp REQUIRED)

# The system plugin. Gazebo finds it through GZ_SIM_SYSTEM_PLUGIN_PATH, set by
# the environment hook below when the workspace is sourced.
# Links the MagisV2 firmware (GPL-3.0-or-later): the plugin binary is GPL.
add_library(pluto_x_vehicle_system SHARED
  src/vehicle_system.cpp
)
target_include_directories(pluto_x_vehicle_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pluto_x_vehicle_system
  pluto_x_core::pluto_x_core
  pluto_x_magisv2::pluto_x_magisv2_flight_controller
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)
target_compile_options(pluto_x_vehicle_system PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Werror)

# The balloon arena (world plugin). Does not link MagisV2.
add_library(pluto_x_arena_system SHARED src/arena_system.cpp)
target_include_directories(pluto_x_arena_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pluto_x_arena_system
  pluto_x_core::pluto_x_core
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
  yaml-cpp
)
target_compile_options(pluto_x_arena_system PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Werror)

install(TARGETS pluto_x_vehicle_system pluto_x_arena_system LIBRARY DESTINATION lib)
install(DIRECTORY models worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/prepare_balloon_mesh.py tools/generate_arena.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
