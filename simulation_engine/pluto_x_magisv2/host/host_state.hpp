// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Shared state between the MagisHost facade (src/magis_host.cpp) and the
// hardware shims (host/hal_*.cpp) that replace the MagisV2 driver layer.
//
// The firmware keeps all of its state in globals, so there is exactly one
// host state per process as well. Values are stored in the units the replaced
// driver functions return, so the production code above them runs unchanged.

#ifndef PLUTO_X_MAGISV2_HOST_STATE_HPP_
#define PLUTO_X_MAGISV2_HOST_STATE_HPP_

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace pluto_x_magisv2::host {

inline constexpr int kMotorCount = 4;

struct HostState {
  /// Simulated time returned by micros()/millis().
  std::uint32_t time_us{0};

  /// Gyro and accelerometer as the ICM-20948 driver would return them after
  /// its register read (sensor frame = FLU body for Pluto X, ALIGN_DEFAULT):
  /// gyro in LSB (16.4 per deg/s), accel specific force in LSB (+acc_1G on
  /// z when level).
  std::array<std::int16_t, 3> gyro_counts{};
  std::array<std::int16_t, 3> accel_counts{};
  /// Magnetometer as the AK09916 driver returns it, BEFORE the firmware's
  /// MAG_AK09916_ALIGN rotation.
  std::array<std::int16_t, 3> mag_counts{};
  /// Barometer as the ICP10111 driver returns it.
  float baro_pressure_pa{101325.0f};
  float baro_temperature_c{25.0f};
  /// INA219 readings in the units bus_voltage()/shunt_voltage() return.
  std::uint16_t bus_voltage_decivolts{0};
  std::int16_t shunt_voltage_mv{0};

  /// Last value written to each motor output by pwmWriteMotor (us).
  std::array<std::uint16_t, kMotorCount> motor_pwm_us{};
  std::uint32_t motor_write_count{0};

  bool initialised{false};
  /// delay()/delayMicroseconds() calls after initialisation (they return
  /// immediately on the host; the count is reported for traceability).
  std::uint32_t delay_calls_after_init{0};
};

HostState& State();

/// Raised by the firmware's fatal paths (failureMode, systemReset). The
/// firmware would halt or reboot; the host stops and reports instead.
class FirmwareHalt : public std::runtime_error {
 public:
  explicit FirmwareHalt(const std::string& what) : std::runtime_error(what) {}
};

}  // namespace pluto_x_magisv2::host

#endif  // PLUTO_X_MAGISV2_HOST_STATE_HPP_
