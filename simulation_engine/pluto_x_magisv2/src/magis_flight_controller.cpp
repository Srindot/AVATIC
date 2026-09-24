// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later

#include "pluto_x_magisv2/magis_flight_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x_magisv2 {
namespace {

constexpr std::int64_t kNanosecondsPerMicrosecond = 1000;
constexpr double kPwmMinUs = 1000.0;
constexpr double kPwmSpanUs = 1000.0;

std::atomic<bool> g_firmware_owned{false};

const pluto_x::MagisV2Params& RequireMagisParams(
    const pluto_x::LegacyStackConfig& config) {
  if (config.flight_controller.type != pluto_x::FlightControllerType::kMagisV2) {
    throw std::invalid_argument(
        "MagisFlightController requires flight_controller.type magisv2");
  }
  return config.flight_controller.magisv2;
}

std::int16_t SaturateToInt16(double value) {
  const double clamped =
      std::clamp(std::round(value),
                 static_cast<double>(std::numeric_limits<std::int16_t>::min()),
                 static_cast<double>(std::numeric_limits<std::int16_t>::max()));
  return static_cast<std::int16_t>(clamped);
}

}  // namespace

MagisFlightController::MagisFlightController(
    const pluto_x::LegacyStackConfig& config)
    : params_(RequireMagisParams(config)),
      gravity_m_s2_(config.vehicle.gravity_m_s2),
      imu_(params_.imu),
      baro_(params_.baro) {
  if (g_firmware_owned.exchange(true) || MagisHost::Instance().initialised()) {
    throw std::logic_error(
        "MagisFlightController: the MagisV2 firmware (process-global state) "
        "is already in use; one instance per process");
  }
  options_.busy_loop_period_us = params_.busy_loop_period_us;
  options_.preload_mag_calibration = params_.preload_mag_calibration;
}

MagisFlightController::~MagisFlightController() {
  // The firmware cannot be re-initialised in this process: ownership is not
  // released, so a second instance still fails loudly.
}

SensorInputs MagisFlightController::MakeSensors(
    const pluto_x::FlightControllerInput& input) {
  SensorInputs s;
  const pluto_x::VehicleStateNed& state = input.truth.state;
  const pluto_x::ImuRawSample imu = imu_.Sample(
      pluto_x::SpecificForceFrd(input.truth.acceleration_ned_m_s2,
                                state.attitude, gravity_m_s2_),
      state.body_rate_frd_rad_s);
  s.gyro_counts = imu.gyro_counts;
  s.accel_counts = imu.accel_counts;

  const pluto_x::Vector3 field_frd_ut =
      pluto_x::frames::RotationFromEulerZyx(state.attitude).transpose() *
      params_.magnetic_field_ned_ut;
  for (int axis = 0; axis < 3; ++axis) {
    s.mag_counts[axis] =
        SaturateToInt16(field_frd_ut[axis] * params_.mag_counts_per_ut);
  }

  const pluto_x::BaroSample baro = baro_.Sample(-state.position_ned_m.z());
  s.baro_pressure_pa = static_cast<float>(baro.pressure_pa);
  s.baro_temperature_c = static_cast<float>(baro.temperature_c);
  s.battery_voltage_v = static_cast<float>(input.battery_voltage_v);
  s.battery_current_a = static_cast<float>(input.battery_current_a);
  return s;
}

pluto_x::FlightControllerOutput MagisFlightController::Step(
    const pluto_x::FlightControllerInput& input) {
  pluto_x::FlightControllerOutput output;
  if (input.time_ns < 0 ||
      input.time_ns / kNanosecondsPerMicrosecond >
          static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::invalid_argument(
        "MagisFlightController: time must be in [0, 2^32) us");
  }
  if (halted_) {
    output.healthy = false;
    return output;
  }
  if (!pluto_x::IsFinite(input.truth.state) ||
      !pluto_x::IsFinite(input.truth.acceleration_ned_m_s2)) {
    // No physical sensor produces NaN; treat as a simulator fault.
    output.healthy = false;
    return output;
  }
  const auto time_us =
      static_cast<std::uint32_t>(input.time_ns / kNanosecondsPerMicrosecond);
  MagisHost& host = MagisHost::Instance();
  sensors_ = MakeSensors(input);
  try {
    if (!host.initialised()) {
      host.Initialise(time_us, sensors_, options_);
    } else {
      host.SetSensors(sensors_);
    }
    if (input.rc && input.rc->sequence != last_rc_sequence_) {
      RcChannelsUs channels{};
      std::copy(input.rc->channels_us.begin(), input.rc->channels_us.end(),
                channels.begin());
      host.SetRc(channels);
      last_rc_sequence_ = input.rc->sequence;
    }
    host.RunUntil(time_us);
  } catch (const std::runtime_error&) {
    halted_ = true;
    output.healthy = false;
    return output;
  }

  pwm_us_ = host.MotorPwm();
  status_ = host.Status();
  for (int i = 0; i < pluto_x::kRotorCount; ++i) {
    output.motor_duty[i] = std::clamp(
        (static_cast<double>(pwm_us_[i]) - kPwmMinUs) / kPwmSpanUs, 0.0, 1.0);
  }
  output.control_step_ran = status_.loop_calls != last_loop_calls_;
  last_loop_calls_ = status_.loop_calls;
  output.armed = status_.armed;
  output.healthy = true;
  pluto_x::FlightControllerTelemetry& telemetry = output.telemetry;
  telemetry.armed = status_.armed;
  telemetry.ok_to_arm = status_.ok_to_arm;
  telemetry.calibrated = status_.calibrated;
  telemetry.angle_mode = status_.angle_mode;
  telemetry.altitude_hold = status_.baro_mode;
  telemetry.roll_deg = status_.roll_decideg * 0.1;
  telemetry.pitch_deg = -status_.pitch_decideg * 0.1;  // firmware: nose down +
  telemetry.heading_deg = status_.heading_deg;
  telemetry.altitude_m = status_.estimated_altitude_cm * 0.01;
  telemetry.battery_v = status_.battery_decivolts * 0.1;
  return output;
}

}  // namespace pluto_x_magisv2
