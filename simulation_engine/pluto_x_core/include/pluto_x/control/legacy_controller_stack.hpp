// Copyright 2026 AVATIC contributors.
//
// One fixed-period step of the kwad.cpp control cascade:
//
//   [position hold only] LegacyPositionController -> roll/pitch/thrust
//   LegacyAttitudeController                      -> body-rate setpoint
//   LegacyRateController                          -> body torques
//   QuadMixer                                     -> rotor speeds, achieved
//                                                    wrench
//
// The stack is transport- and simulator-agnostic: it knows nothing about
// ROS 2 or Gazebo. The caller supplies the state (NED/FRD), the measured
// body angular acceleration and the latched pilot command, and is
// responsible for calling Step() exactly once per configured period.

#ifndef PLUTO_X_CONTROL_LEGACY_CONTROLLER_STACK_HPP_
#define PLUTO_X_CONTROL_LEGACY_CONTROLLER_STACK_HPP_

#include <optional>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/legacy_cascade.hpp"
#include "pluto_x/control/quad_mixer.hpp"
#include "pluto_x/control/pilot_command.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

struct ControllerInput {
  VehicleStateNed state;
  /// Measured body angular acceleration (legacy p_dot, q_dot, r_dot), used
  /// by the rate-loop derivative term.
  Vector3 body_angular_accel_frd_rad_s2{Vector3::Zero()};
  PilotCommand pilot;
};

enum class ControllerStatus {
  kOk,
  /// Input contained NaN/Inf. Output commands minimum thrust, zero torque.
  kInvalidInput,
};

struct ControllerOutput {
  ControllerStatus status{ControllerStatus::kOk};
  FlightMode mode{FlightMode::kManualAttitude};
  EulerAnglesZyx attitude_setpoint;
  Vector3 body_rate_setpoint_frd_rad_s{Vector3::Zero()};
  /// Wrench requested by the loops before rotor saturation.
  BodyWrenchCommand requested;
  /// Rotor speeds and the wrench they achieve.
  MixerOutput mixer;
};

class LegacyControllerStack {
 public:
  /// Precondition: config has passed ValidateLegacyStackConfig().
  explicit LegacyControllerStack(const LegacyStackConfig& config);

  ControllerOutput Step(const ControllerInput& input);

  /// Clears every integrator and the latched heading (not called by
  /// Step(); legacy kwad.cpp only zeroed integrators once at start-up).
  void Reset();

  double period_s() const { return period_s_; }
  const Vector3& position_hold_target_ned_m() const {
    return position_hold_target_ned_m_;
  }

 private:
  ControllerOutput MakeSafeOutput(FlightMode mode) const;

  double period_s_;
  bool hold_initial_heading_;
  /// Latched on the first valid Step() when hold_initial_heading_.
  std::optional<double> yaw_setpoint_rad_;
  LegacyActuationLimits limits_;
  Vector3 position_hold_target_ned_m_;
  LegacyPositionController position_controller_;
  LegacyAttitudeController attitude_controller_;
  LegacyRateController rate_controller_;
  QuadMixer mixer_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_LEGACY_CONTROLLER_STACK_HPP_
