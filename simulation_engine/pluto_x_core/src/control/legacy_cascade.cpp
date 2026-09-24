// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/legacy_cascade.hpp"

#include <cmath>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {

// ---------------------------------------------------------------------------
// Position loop
// ---------------------------------------------------------------------------

LegacyPositionController::LegacyPositionController(
    const LegacyPositionControllerParams& params,
    const LegacyActuationLimits& limits, const LegacyVehicleParams& vehicle)
    : params_(params),
      limits_(limits),
      weight_n_(vehicle.mass_kg * vehicle.gravity_m_s2),
      // Legacy: the north integral term is clamped to +/-theta_max, the east
      // one to +/-phi_max and the down one to [u1_min, u1_max].
      north_term_(params.north, -params.pitch_limit_rad, params.pitch_limit_rad),
      east_term_(params.east, -params.roll_limit_rad, params.roll_limit_rad),
      down_term_(params.down, limits.thrust_min_n, limits.thrust_max_n) {
  RequireFinite(weight_n_, "vehicle weight");
}

PositionControllerOutput LegacyPositionController::Update(
    const VehicleStateNed& state, const Vector3& target_ned_m,
    double timestep_s) {
  if (!IsFinite(target_ned_m)) {
    throw std::invalid_argument("position target must be finite");
  }
  const EulerAnglesZyx yaw_only{0.0, 0.0, state.attitude.yaw_rad};
  const EulerAnglesZyx& state_rotation =
      (params_.error_frame == PositionErrorFrame::kLegacyFullAttitude)
          ? state.attitude
          : yaw_only;
  const Vector3 target_local = frames::WorldToBodyZyx(target_ned_m, yaw_only);
  const Vector3 position_body =
      frames::WorldToBodyZyx(state.position_ned_m, state_rotation);
  const Vector3 velocity_body =
      frames::WorldToBodyZyx(state.velocity_ned_m_s, state_rotation);

  PositionControllerOutput output;

  const double north_error = target_local.x() - position_body.x();
  const double north_raw =
      north_term_.Update(north_error, velocity_body.x(), timestep_s);
  // Nose-down (negative pitch, FRD) accelerates north.
  output.pitch_setpoint_rad =
      ClampSymmetric(-north_raw, params_.pitch_limit_rad);

  const double east_error = target_local.y() - position_body.y();
  const double east_raw =
      east_term_.Update(east_error, velocity_body.y(), timestep_s);
  output.roll_setpoint_rad = ClampSymmetric(east_raw, params_.roll_limit_rad);

  const double down_error = target_local.z() - position_body.z();
  const double down_raw = down_term_.Update(
      down_error, state.velocity_ned_m_s.z(), timestep_s);
  const double tilt_cosine =
      std::cos(state.attitude.pitch_rad) * std::cos(state.attitude.roll_rad);
  if (tilt_cosine > 0.0) {
    const double thrust_n = (weight_n_ - down_raw) / tilt_cosine;
    output.thrust_n =
        Clamp(thrust_n, limits_.thrust_min_n, limits_.thrust_max_n);
  } else {
    output.thrust_n = limits_.thrust_min_n;
  }
  return output;
}

void LegacyPositionController::Reset() {
  north_term_.Reset();
  east_term_.Reset();
  down_term_.Reset();
}

// ---------------------------------------------------------------------------
// Attitude loop
// ---------------------------------------------------------------------------

LegacyAttitudeController::LegacyAttitudeController(
    const LegacyAttitudeControllerParams& params)
    : params_(params),
      roll_term_(params.roll, -params.body_rate_limit_rad_s.x(),
                 params.body_rate_limit_rad_s.x()),
      pitch_term_(params.pitch, -params.body_rate_limit_rad_s.y(),
                  params.body_rate_limit_rad_s.y()),
      yaw_term_(params.yaw, -params.body_rate_limit_rad_s.z(),
                params.body_rate_limit_rad_s.z()) {}

Vector3 LegacyAttitudeController::Update(
    const VehicleStateNed& state, const EulerAnglesZyx& attitude_setpoint,
    double timestep_s) {
  if (!IsFinite(attitude_setpoint)) {
    throw std::invalid_argument("attitude setpoint must be finite");
  }
  const Vector3& rate = state.body_rate_frd_rad_s;
  const Vector3& limit = params_.body_rate_limit_rad_s;

  // Legacy derivative signal for the angle loops is the body rate.
  const double roll_raw = roll_term_.Update(
      attitude_setpoint.roll_rad - state.attitude.roll_rad, rate.x(),
      timestep_s);
  const double pitch_raw = pitch_term_.Update(
      attitude_setpoint.pitch_rad - state.attitude.pitch_rad, rate.y(),
      timestep_s);
  const double yaw_raw = yaw_term_.Update(
      attitude_setpoint.yaw_rad - state.attitude.yaw_rad, rate.z(),
      timestep_s);

  return Vector3(ClampSymmetric(roll_raw, limit.x()),
                 ClampSymmetric(pitch_raw, limit.y()),
                 ClampSymmetric(yaw_raw, limit.z()));
}

void LegacyAttitudeController::Reset() {
  roll_term_.Reset();
  pitch_term_.Reset();
  yaw_term_.Reset();
}

// ---------------------------------------------------------------------------
// Rate loop
// ---------------------------------------------------------------------------

LegacyRateController::LegacyRateController(
    const LegacyRateControllerParams& params,
    const LegacyActuationLimits& limits)
    : torque_limit_n_m_(limits.torque_limit_n_m),
      roll_term_(params.roll, -limits.torque_limit_n_m.x(),
                 limits.torque_limit_n_m.x()),
      pitch_term_(params.pitch, -limits.torque_limit_n_m.y(),
                  limits.torque_limit_n_m.y()),
      yaw_term_(params.yaw, -limits.torque_limit_n_m.z(),
                limits.torque_limit_n_m.z()) {}

Vector3 LegacyRateController::Update(
    const Vector3& body_rate_frd_rad_s,
    const Vector3& body_angular_accel_frd_rad_s2,
    const Vector3& body_rate_setpoint_frd_rad_s, double timestep_s) {
  if (!IsFinite(body_rate_frd_rad_s) ||
      !IsFinite(body_angular_accel_frd_rad_s2) ||
      !IsFinite(body_rate_setpoint_frd_rad_s)) {
    throw std::invalid_argument("rate controller inputs must be finite");
  }
  const Vector3 error = body_rate_setpoint_frd_rad_s - body_rate_frd_rad_s;
  const Vector3& accel = body_angular_accel_frd_rad_s2;
  const double roll_raw = roll_term_.Update(error.x(), accel.x(), timestep_s);
  const double pitch_raw =
      pitch_term_.Update(error.y(), accel.y(), timestep_s);
  const double yaw_raw = yaw_term_.Update(error.z(), accel.z(), timestep_s);
  return Vector3(ClampSymmetric(roll_raw, torque_limit_n_m_.x()),
                 ClampSymmetric(pitch_raw, torque_limit_n_m_.y()),
                 ClampSymmetric(yaw_raw, torque_limit_n_m_.z()));
}

void LegacyRateController::Reset() {
  roll_term_.Reset();
  pitch_term_.Reset();
  yaw_term_.Reset();
}

}  // namespace pluto_x
