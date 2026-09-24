// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/legacy_controller_stack.hpp"

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {

LegacyControllerStack::LegacyControllerStack(const LegacyStackConfig& config)
    : period_s_(config.controller.period_s),
      hold_initial_heading_(config.pilot.hold_initial_heading),
      limits_(config.actuation_limits),
      position_hold_target_ned_m_(
          frames::SwapNedEnu(config.pilot.position_hold_target_enu_m)),
      position_controller_(config.controller.position,
                           config.actuation_limits, config.vehicle),
      attitude_controller_(config.controller.attitude),
      rate_controller_(config.controller.rate, config.actuation_limits),
      mixer_(config.rotor, config.vehicle.arm_length_m) {}

ControllerOutput LegacyControllerStack::Step(const ControllerInput& input) {
  if (!IsFinite(input.state) ||
      !IsFinite(input.body_angular_accel_frd_rad_s2) ||
      !IsFinite(input.pilot)) {
    return MakeSafeOutput(input.pilot.mode);
  }

  ControllerOutput output;
  output.mode = input.pilot.mode;

  // Heading: latched at the first valid step (or 0 = north, as in kwad.cpp,
  // which had no yaw stick).
  if (!yaw_setpoint_rad_) {
    yaw_setpoint_rad_ =
        hold_initial_heading_ ? input.state.attitude.yaw_rad : 0.0;
  }

  // Stage 1: thrust and roll/pitch setpoint.
  if (input.pilot.mode == FlightMode::kPositionHold) {
    const PositionControllerOutput position = position_controller_.Update(
        input.state, position_hold_target_ned_m_, period_s_);
    output.requested.thrust_n = position.thrust_n;
    output.attitude_setpoint.roll_rad = position.roll_setpoint_rad;
    output.attitude_setpoint.pitch_rad = position.pitch_setpoint_rad;
  } else {
    output.requested.thrust_n = input.pilot.thrust_n;
    output.attitude_setpoint.roll_rad = input.pilot.roll_setpoint_rad;
    output.attitude_setpoint.pitch_rad = input.pilot.pitch_setpoint_rad;
  }
  output.attitude_setpoint.yaw_rad = *yaw_setpoint_rad_;

  // Stage 2: attitude -> body-rate setpoint.
  output.body_rate_setpoint_frd_rad_s = attitude_controller_.Update(
      input.state, output.attitude_setpoint, period_s_);

  // Stage 3: body rate -> torque.
  output.requested.torque_frd_n_m = rate_controller_.Update(
      input.state.body_rate_frd_rad_s, input.body_angular_accel_frd_rad_s2,
      output.body_rate_setpoint_frd_rad_s, period_s_);

  // Stage 4: mixer.
  output.mixer = mixer_.Mix(output.requested);
  return output;
}

void LegacyControllerStack::Reset() {
  yaw_setpoint_rad_.reset();
  position_controller_.Reset();
  attitude_controller_.Reset();
  rate_controller_.Reset();
}

ControllerOutput LegacyControllerStack::MakeSafeOutput(FlightMode mode) const {
  ControllerOutput output;
  output.status = ControllerStatus::kInvalidInput;
  output.mode = mode;
  output.requested.thrust_n = limits_.thrust_min_n;
  output.requested.torque_frd_n_m = Vector3::Zero();
  output.mixer = mixer_.Mix(output.requested);
  return output;
}

}  // namespace pluto_x
