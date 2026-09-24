// Copyright 2026 AVATIC contributors.
//
// Vehicle state and actuation command types shared by the legacy controller
// stack and the legacy dynamics model. All in NED world / FRD body.

#ifndef PLUTO_X_DYNAMICS_VEHICLE_STATE_HPP_
#define PLUTO_X_DYNAMICS_VEHICLE_STATE_HPP_

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

/// Full rigid-body state as used by kwad.cpp (x, y, z, x_dot, ..., phi,
/// theta, psi, p, q, r).
struct VehicleStateNed {
  Vector3 position_ned_m{Vector3::Zero()};
  Vector3 velocity_ned_m_s{Vector3::Zero()};
  /// Attitude of FRD body relative to NED world, Z-Y-X Euler.
  EulerAnglesZyx attitude;
  /// Angular velocity of the body relative to the world, in FRD (p, q, r).
  Vector3 body_rate_frd_rad_s{Vector3::Zero()};
};

bool IsFinite(const VehicleStateNed& state);

/// Collective thrust and body torques (legacy u1, u2, u3, u4).
///
/// thrust_n acts along -z_FRD (upwards when level).
/// torque_frd_n_m = (u2, u3, u4) as produced by the legacy rate loop/mixer.
/// NOTE: the legacy dynamics multiply u2 and u3 by the arm length again
/// before applying them; see ComputeLegacyExternalWrench().
struct BodyWrenchCommand {
  double thrust_n{0.0};
  Vector3 torque_frd_n_m{Vector3::Zero()};
};

bool IsFinite(const BodyWrenchCommand& command);

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_VEHICLE_STATE_HPP_
