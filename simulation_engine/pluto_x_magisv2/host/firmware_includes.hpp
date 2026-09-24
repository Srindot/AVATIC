// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Firmware headers for the host code, in the order the firmware's own
// translation units include them. MagisV2 headers are not self-contained:
// each relies on earlier includes. This block is mw.cpp's include list
// (lines 23-115 at upstream 5982032), followed by the driver headers that
// sensors/initialisation.cpp adds, and the remaining driver headers whose
// functions the host shims replace.

#ifndef PLUTO_X_MAGISV2_FIRMWARE_INCLUDES_HPP_
#define PLUTO_X_MAGISV2_FIRMWARE_INCLUDES_HPP_

// --- mw.cpp -----------------------------------------------------------------
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "platform.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/ranging_vl53l1x.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/flashfs.h"
#include "io/oled_display.h"
#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/crsf.h"
#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"
#include "flight/opticflow.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "mw.h"
#include "API/API-Utils.h"
#include "API/Scheduler-Timer.h"
#include "API/RC-Interface.h"
#include "API/FC-Control.h"
#include "API/FC-Data.h"
#include "API/Motor.h"
#include "PlutoPilot.h"
#include "API/XRanging.h"
#include "API/Localisation.h"
#include "API/RxConfig.h"
#include "command/command.h"
#include "command/localisationCommand.h"
#include "drivers/opticflow_paw3903.h"

// --- sensors/initialisation.cpp additions -------------------------------------
#include "build_config.h"
#include "drivers/exti.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_icm20948.h"
#include "drivers/bus_spi.h"
#include "drivers/barometer.h"
#include "drivers/barometer_icp10111.h"
#include "drivers/compass_ak09916.h"
#include "sensors/initialisation.h"

// --- drivers replaced by host shims -----------------------------------------------
#include "drivers/bus_i2c.h"
#include "drivers/display_ug2864hsweg01.h"
#include "drivers/ina219.h"
#include "drivers/pwm_mapping.h"
#include "drivers/serial_uart.h"

#endif  // PLUTO_X_MAGISV2_FIRMWARE_INCLUDES_HPP_
