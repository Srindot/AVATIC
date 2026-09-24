// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Replaces the INA219 battery monitor (drivers/ina219.c) and the motor PWM
// output driver (drivers/pwm_output.cpp).
//
// INA219: bus_voltage() returns battery voltage in 0.1 V steps (that is what
// the real driver's arithmetic yields); shunt_voltage() returns the value
// that the firmware's own conversion in sensors/battery.cpp,
//   mA = (shunt / 10) / INA219_SHUNT_RESISTOR_MILLIOHM,
// maps back to the simulated current. See MagisHost::SetBattery.
//
// Motors: pwmWriteMotor records the 1000..2000 us value per motor index
// (M0..M3); the simulator reads them through MagisHost::motors().

#include "host_state.hpp"

#include "firmware_includes.hpp"

using pluto_x_magisv2::host::kMotorCount;
using pluto_x_magisv2::host::State;

uint16_t bus_voltage(void) { return State().bus_voltage_decivolts; }

int16_t shunt_voltage(void) { return State().shunt_voltage_mv; }

void pwmWriteMotor(uint8_t index, uint16_t value) {
  if (index < kMotorCount) {
    State().motor_pwm_us[index] = value;
    ++State().motor_write_count;
  }
}

void pwmShutdownPulsesForAllMotors(uint8_t motor_count) {
  for (uint8_t i = 0; i < motor_count && i < kMotorCount; ++i) {
    State().motor_pwm_us[i] = 0;
  }
}

void pwmCompleteOneshotMotorUpdate(uint8_t /*motorCount*/) {}

// Only reached from the user-code motor/PWM APIs (API-Src/Motor.cpp,
// Peripheral-PWM.cpp), which configure extra STM32 timer channels.
pwmOutputPort_t* pwmOutConfig(const timerHardware_t* /*timerHardware*/,
                              uint8_t /*mhz*/, uint16_t /*period*/,
                              uint16_t /*value*/) {
  static pwmOutputPort_t inert_port;
  return &inert_port;
}
