cmake_minimum_required(VERSION 3.16)
project(pluto_x_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PLUTO_X_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion
                     -Wno-sign-conversion -Werror)

add_library(${PROJECT_NAME} SHARED
  src/common/backward_difference.cpp
  src/common/frames.cpp
  src/common/validation.cpp
  src/config/config_loader.cpp
  src/control/legacy_cascade.cpp
  src/control/legacy_controller_stack.cpp
  src/control/legacy_flight_controller.cpp
  src/control/legacy_pid_term.cpp
  src/control/quad_mixer.cpp
  src/control/pilot_command.cpp
  src/dynamics/aerodynamic_drag.cpp
  src/dynamics/battery_model.cpp
  src/dynamics/enu_adapter.cpp
  src/dynamics/legacy_dynamics.cpp
  src/dynamics/propulsion_model.cpp
  src/dynamics/rotor_actuator.cpp
  src/dynamics/vehicle_state.cpp
  src/environment/wind_model.cpp
  src/sensors/baro_model.cpp
  src/arena/balloon_shape.cpp
  src/arena/arena_scoring.cpp
  src/sensors/imu_model.cpp
  src/sensors/state_error_model.cpp
  src/simulation/simulated_vehicle.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE yaml-cpp)
target_compile_options(${PROJECT_NAME} PRIVATE ${PLUTO_X_WARNINGS})

add_executable(legacy_reference_sim tools/legacy_reference_sim.cpp)
target_link_libraries(legacy_reference_sim PRIVATE ${PROJECT_NAME})
target_compile_options(legacy_reference_sim PRIVATE ${PLUTO_X_WARNINGS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS legacy_reference_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test_aerodynamic_drag.cpp
    test/test_arena.cpp
    test/test_backward_difference.cpp
    test/test_battery_model.cpp
    test/test_config_loader.cpp
    test/test_delay_line.cpp
    test/test_enu_adapter.cpp
    test/test_frames.cpp
    test/test_legacy_cascade.cpp
    test/test_legacy_controller_stack.cpp
    test/test_legacy_dynamics.cpp
    test/test_legacy_pid_term.cpp
    test/test_quad_mixer.cpp
    test/test_pilot_command.cpp
    test/test_propulsion_model.cpp
    test/test_rotor_actuator.cpp
    test/test_simulated_vehicle.cpp
    test/test_sensor_models.cpp
    test/test_state_error_model.cpp
    test/test_validation.cpp
    test/test_wind_model.cpp
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_tests PRIVATE
    PLUTO_X_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/legacy_kwad.yaml"
    PLUTO_X_ESTIMATED_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/pluto_x_estimated.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
