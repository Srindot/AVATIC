// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/legacy_flight_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;

}  // namespace

LegacyFlightController::LegacyFlightController(const LegacyStackConfig& config)
    : period_s_(config.controller.period_s),
      period_ns_(static_cast<std::int64_t>(
          std::llround(config.controller.period_s * kNanosecondsPerSecond))),
      max_rotor_speed_rad_s_(std::sqrt(config.rotor.speed_squared_max_rad2_s2)),
      stack_(config),
      state_error_(config.state_error) {
  pilot_.mode = config.pilot.initial_mode;
  pilot_.thrust_n = config.pilot.hover_thrust_n;
  last_output_.mode = config.pilot.initial_mode;
}

FlightControllerOutput LegacyFlightController::Step(
    const FlightControllerInput& input) {
  if (!next_tick_ns_) {
    next_tick_ns_ = input.time_ns;
  }
  FlightControllerOutput output = last_command_;
  output.control_step_ran = false;
  if (input.time_ns < *next_tick_ns_) {
    return output;
  }
  *next_tick_ns_ += period_ns_;

  ControllerInput stack_input;
  stack_input.pilot = pilot_;
  if (IsFinite(input.truth.state)) {
    stack_input.state = state_error_.Measure(input.truth.state);
    stack_input.body_angular_accel_frd_rad_s2 = angular_accel_estimator_.Update(
        stack_input.state.body_rate_frd_rad_s, period_s_);
  } else {
    stack_input.state = input.truth.state;  // stack reports kInvalidInput
  }
  last_output_ = stack_.Step(stack_input);

  for (int i = 0; i < kRotorCount; ++i) {
    output.motor_duty[i] =
        std::clamp(last_output_.mixer.rotor_speed_rad_s[i] /
                       max_rotor_speed_rad_s_, 0.0, 1.0);
  }
  output.control_step_ran = true;
  output.armed = true;  // the legacy stack has no arming logic
  output.healthy = last_output_.status == ControllerStatus::kOk;
  // The legacy stack is "armed" from t = 0 and estimates by the state error
  // model.
  FlightControllerTelemetry& telemetry = output.telemetry;
  telemetry.armed = telemetry.ok_to_arm = telemetry.calibrated = true;
  telemetry.angle_mode = pilot_.mode == FlightMode::kManualAttitude;
  telemetry.altitude_hold = pilot_.mode == FlightMode::kPositionHold;
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  telemetry.roll_deg = stack_input.state.attitude.roll_rad * kRadToDeg;
  telemetry.pitch_deg = stack_input.state.attitude.pitch_rad * kRadToDeg;
  const double heading_deg = std::fmod(
      stack_input.state.attitude.yaw_rad * kRadToDeg, 360.0);
  telemetry.heading_deg = heading_deg < 0.0 ? heading_deg + 360.0 : heading_deg;
  telemetry.altitude_m = -stack_input.state.position_ned_m.z();
  telemetry.battery_v = input.battery_voltage_v;
  last_command_ = output;
  return output;
}

}  // namespace pluto_x
