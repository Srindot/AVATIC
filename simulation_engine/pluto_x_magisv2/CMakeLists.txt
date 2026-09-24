cmake_minimum_required(VERSION 3.16)
project(pluto_x_magisv2 C CXX)

# Host build of the MagisV2 flight-controller firmware (GPL-3.0-or-later).
#
# The vendored firmware in third_party/magisv2 is never edited. At configure
# time it is copied into the build tree and the patches in patches/ are
# applied to the copy (each patch is guarded by MAGIS_HOST and documented).
# The production flight-control modules are compiled unmodified from that
# copy; the hardware driver layer (STM32 peripherals, I2C sensors, PWM
# timers, flash) is replaced by the host shims in host/.

find_package(ament_cmake REQUIRED)
find_package(pluto_x_core REQUIRED)

set(MAGIS_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/magisv2"
    CACHE PATH "Vendored MagisV2 firmware (unmodified upstream copy)")
set(MAGIS_ROOT "${CMAKE_CURRENT_BINARY_DIR}/magisv2")
set(MAGIS_SRC "${MAGIS_ROOT}/src/main")

# --- copy + patch --------------------------------------------------------
file(GLOB MAGIS_PATCHES "${CMAKE_CURRENT_SOURCE_DIR}/patches/*.patch")
list(SORT MAGIS_PATCHES)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${MAGIS_PATCHES})
file(REMOVE_RECURSE "${MAGIS_ROOT}")
file(COPY "${MAGIS_VENDOR_DIR}/src" "${MAGIS_VENDOR_DIR}/lib"
          "${MAGIS_VENDOR_DIR}/PlutoPilot.cpp" "${MAGIS_VENDOR_DIR}/PlutoPilot.h"
     DESTINATION "${MAGIS_ROOT}")
foreach(patch_file ${MAGIS_PATCHES})
  execute_process(
    COMMAND patch -p1 --forward --silent -i "${patch_file}"
    WORKING_DIRECTORY "${MAGIS_ROOT}"
    RESULT_VARIABLE patch_result)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR "failed to apply ${patch_file}")
  endif()
  message(STATUS "MagisV2: applied ${patch_file}")
endforeach()

# --- production firmware modules (compiled unmodified) -------------------
# Same lists as the firmware Makefile (PRIMUSX2 target, BUILD_TYPE=BIN) minus
# main.cpp (replaced by the host init in src/magis_host.cpp) and minus every
# hardware driver, startup, CMSIS and StdPeriph file.
set(MAGIS_FIRMWARE_SOURCES
  build_config.cpp debug.cpp mw.cpp version.cpp
  config/config.cpp config/runtime_config.cpp
  common/maths.cpp common/printf.cpp common/typeconversion.cpp common/encoding.cpp
  flight/altitudehold.cpp flight/failsafe.cpp flight/pid.cpp flight/imu.cpp
  flight/mixer.cpp flight/lowpass.cpp flight/filter.cpp flight/navigation.cpp
  flight/gps_conversion.c flight/motor.cpp
  flight/acrobats.cpp flight/posControl.cpp flight/posEstimate.cpp flight/opticflow.cpp
  io/beeper.cpp io/oled_display.c io/rc_controls.cpp io/rc_curves.cpp io/serial.cpp
  io/serial_1wire.cpp io/serial_cli.cpp io/serial_msp.cpp io/statusindicator.cpp
  io/flashfs.cpp io/gps.cpp
  rx/rx.cpp rx/pwm.c rx/msp.c rx/sbus.c rx/sumd.c rx/sumh.c rx/spektrum.c rx/xbus.cpp rx/crsf.c
  sensors/acceleration.cpp sensors/battery.cpp sensors/boardalignment.cpp
  sensors/compass.cpp sensors/gyro.cpp sensors/initialisation.cpp sensors/barometer.cpp
  blackbox/blackbox.cpp blackbox/blackbox_io.cpp
  command/command.cpp command/localisationCommand.cpp
  drivers/serial.cpp
  API-Src/Status-LED.cpp API-Src/Peripheral-ADC.cpp API-Src/Peripheral-PWM.cpp
  API-Src/Peripheral-GPIO.cpp API-Src/Serial-IO-Uart.cpp API-Src/Serial-IO-I2C.cpp
  API-Src/Serial-IO-SPI.cpp API-Src/BMS.cpp API-Src/FC-Config-PID.cpp
  API-Src/FC-Config-Setpoints.cpp API-Src/FC-Control-Status.cpp
  API-Src/FC-Control-Command.cpp API-Src/FC-Data-Sensors.cpp
  API-Src/FC-Data-Estimate.cpp API-Src/RC-Interface.cpp API-Src/Scheduler-Timer.cpp
  API-Src/Debugging.cpp API-Src/XRanging.cpp API-Src/Motor.cpp API-Src/API-Utils.cpp
  API-Src/RxConfig.cpp API-Src/Oled.cpp API-Src/Localisation.cpp
)
list(TRANSFORM MAGIS_FIRMWARE_SOURCES PREPEND "${MAGIS_SRC}/")
list(APPEND MAGIS_FIRMWARE_SOURCES "${MAGIS_ROOT}/PlutoPilot.cpp")

set(MAGIS_INCLUDE_DIRS
  "${MAGIS_SRC}" "${MAGIS_ROOT}" "${MAGIS_SRC}/target/PRIMUSX2" "${MAGIS_SRC}/vcp"
  "${MAGIS_ROOT}/lib/main/STM32F30x_StdPeriph_Driver/inc"
  "${MAGIS_ROOT}/lib/main/CMSIS/CM1/CoreSupport"
  "${MAGIS_ROOT}/lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x"
  "${MAGIS_ROOT}/lib/main/STM32_USB-FS-Device_Driver/inc"
  "${MAGIS_ROOT}/lib/main/VL53L0X_API/core/inc" "${MAGIS_ROOT}/lib/main/VL53L0X_API/platform/inc"
  "${MAGIS_ROOT}/lib/main/VL53L1X_API/core/inc" "${MAGIS_ROOT}/lib/main/VL53L1X_API/platform/inc")

# Same target defines as the firmware Makefile, plus MAGIS_HOST for patches.
set(MAGIS_DEFINES STM32F303xC STM32F303 USE_STDPERIPH_DRIVER PRIMUSX2 FLASH_SIZE=256
  __FORKNAME__="MAGISV2" __TARGET__="PRIMUSX2" __FW_VER__="host"
  __API_VER__="host" __PROJECT__="HOST" __BUILD_DATE__="host" __BUILD_TIME__="host"
  MAGIS_HOST=1)

# ARM ABI assumptions the firmware relies on: plain char is unsigned, float
# literals are single precision. -fpermissive: API-Src/Peripheral-ADC.cpp and
# Peripheral-PWM.cpp cast peripheral register pointers to uint32_t (a 32-bit
# MCU assumption); those peripheral APIs are only reached from on-board user
# code driving STM32 peripherals, never from the flight-control path.
# Warnings from upstream code are silenced for the firmware objects only.
set(MAGIS_FIRMWARE_FLAGS -funsigned-char -fsingle-precision-constant
  -fno-strict-aliasing -w $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17 -fpermissive>
  $<$<COMPILE_LANGUAGE:C>:-std=gnu17 -fexceptions>)

add_library(magisv2_firmware OBJECT ${MAGIS_FIRMWARE_SOURCES})
target_include_directories(magisv2_firmware PUBLIC ${MAGIS_INCLUDE_DIRS})
target_compile_definitions(magisv2_firmware PUBLIC ${MAGIS_DEFINES})
target_compile_options(magisv2_firmware PRIVATE ${MAGIS_FIRMWARE_FLAGS})
set_target_properties(magisv2_firmware PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- host HAL shims + facade ----------------------------------------------
file(GLOB MAGIS_HOST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/host/*.cpp")
add_library(pluto_x_magisv2 SHARED
  ${MAGIS_HOST_SOURCES}
  src/magis_host.cpp
  $<TARGET_OBJECTS:magisv2_firmware>)
target_include_directories(pluto_x_magisv2
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${MAGIS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/host)
target_compile_definitions(pluto_x_magisv2 PRIVATE ${MAGIS_DEFINES})
target_compile_options(pluto_x_magisv2 PRIVATE -funsigned-char
  -fsingle-precision-constant $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
target_link_options(pluto_x_magisv2 PRIVATE -Wl,--no-undefined)
target_link_libraries(pluto_x_magisv2 PRIVATE m)

# The firmware as a pluto_x::FlightController (sensor models, RC, PWM->duty).
add_library(pluto_x_magisv2_flight_controller SHARED
  src/magis_flight_controller.cpp)
target_include_directories(pluto_x_magisv2_flight_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(pluto_x_magisv2_flight_controller PUBLIC cxx_std_17)
target_compile_options(pluto_x_magisv2_flight_controller PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(pluto_x_magisv2_flight_controller
  PUBLIC pluto_x_magisv2 pluto_x_core::pluto_x_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS pluto_x_magisv2 pluto_x_magisv2_flight_controller EXPORT export_pluto_x_magisv2
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tests test/test_magis_host.cpp)
  target_link_libraries(${PROJECT_NAME}_tests pluto_x_magisv2)
  # Separate process: the firmware is a per-process singleton.
  ament_add_gtest(${PROJECT_NAME}_flight_controller_tests
    test/test_magis_flight_controller.cpp
    TIMEOUT 300)
  target_link_libraries(${PROJECT_NAME}_flight_controller_tests
    pluto_x_magisv2_flight_controller)
  target_compile_definitions(${PROJECT_NAME}_flight_controller_tests PRIVATE
    PLUTO_X_ESTIMATED_CONFIG="${pluto_x_core_DIR}/../config/pluto_x_estimated.yaml")
endif()

ament_export_targets(export_pluto_x_magisv2 HAS_LIBRARY_TARGET)
ament_export_dependencies(pluto_x_core)
ament_package()
