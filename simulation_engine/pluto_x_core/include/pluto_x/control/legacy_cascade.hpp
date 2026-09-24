// Copyright 2026 AVATIC contributors.
//
// The three cascaded loops of kwad.cpp, one class per loop:
//
//   LegacyPositionController  PID_position()  position -> roll/pitch/thrust
//   LegacyAttitudeController  PID_attitude()  attitude -> body-rate setpoint
//   LegacyRateController      PID_rate()      body rate -> body torques
//
// All quantities NED/FRD. Each class owns only its integrators.

#ifndef PLUTO_X_CONTROL_LEGACY_CASCADE_HPP_
#define PLUTO_X_CONTROL_LEGACY_CASCADE_HPP_

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/legacy_pid_term.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

struct PositionControllerOutput {
  double roll_setpoint_rad{0.0};
  double pitch_setpoint_rad{0.0};
  double thrust_n{0.0};
};

/// Legacy PID_position().
///
/// Faithfully reproduced (including questionable choices):
///  * With PositionErrorFrame::kLegacyFullAttitude the target is rotated
///    into a yaw-only frame but position and velocity are rotated by the
///    FULL attitude before the error is formed (kwad.cpp behaviour, unstable
///    above real ground). kYawOnly rotates all three by yaw only.
///  * North/east derivative terms use body-frame velocity; the down
///    derivative uses world-frame vertical velocity.
///  * Thrust = (m g - pid_down) / (cos(roll) cos(pitch)).
///
/// Fixed relative to kwad.cpp:
///  * kwad.cpp overwrote the global target with its rotated copy every step,
///    so any non-zero yaw rotated the target repeatedly. The target is now
///    an input and is never mutated.
///  * If cos(roll) cos(pitch) <= 0 (vehicle horizontal or inverted) the
///    legacy division is undefined; minimum thrust is commanded instead.
class LegacyPositionController {
 public:
  LegacyPositionController(const LegacyPositionControllerParams& params,
                           const LegacyActuationLimits& limits,
                           const LegacyVehicleParams& vehicle);

  PositionControllerOutput Update(const VehicleStateNed& state,
                                  const Vector3& target_ned_m,
                                  double timestep_s);
  void Reset();

 private:
  LegacyPositionControllerParams params_;
  LegacyActuationLimits limits_;
  double weight_n_;
  LegacyPidTerm north_term_;
  LegacyPidTerm east_term_;
  LegacyPidTerm down_term_;
};

/// Legacy PID_attitude(). Angle errors are NOT wrapped (legacy behaviour):
/// a yaw near +/-pi against a 0 setpoint produces a large error.
class LegacyAttitudeController {
 public:
  explicit LegacyAttitudeController(
      const LegacyAttitudeControllerParams& params);

  /// Returns the body-rate setpoint (p, q, r) in FRD.
  Vector3 Update(const VehicleStateNed& state,
                 const EulerAnglesZyx& attitude_setpoint, double timestep_s);
  void Reset();

 private:
  LegacyAttitudeControllerParams params_;
  LegacyPidTerm roll_term_;
  LegacyPidTerm pitch_term_;
  LegacyPidTerm yaw_term_;
};

/// Legacy PID_rate(). The derivative term multiplies the *measured angular
/// acceleration* (kwad.cpp: cd = p_kd * p_dot).
class LegacyRateController {
 public:
  LegacyRateController(const LegacyRateControllerParams& params,
                       const LegacyActuationLimits& limits);

  /// Returns the torque command (u2, u3, u4) in FRD.
  Vector3 Update(const Vector3& body_rate_frd_rad_s,
                 const Vector3& body_angular_accel_frd_rad_s2,
                 const Vector3& body_rate_setpoint_frd_rad_s,
                 double timestep_s);
  void Reset();

 private:
  Vector3 torque_limit_n_m_;
  LegacyPidTerm roll_term_;
  LegacyPidTerm pitch_term_;
  LegacyPidTerm yaw_term_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_LEGACY_CASCADE_HPP_
