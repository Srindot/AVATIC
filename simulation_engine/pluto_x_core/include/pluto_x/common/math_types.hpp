// Copyright 2026 AVATIC contributors.
//
// Common linear-algebra aliases used across pluto_x_core.
//
// Vector quantities carry their frame and unit in the variable name
// (e.g. `velocity_ned_m_s`) because Eigen types cannot encode them.

#ifndef PLUTO_X_COMMON_MATH_TYPES_HPP_
#define PLUTO_X_COMMON_MATH_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pluto_x {

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

/// Euler angles, aerospace Z-Y-X (yaw-pitch-roll) convention.
///
/// The rotation from body to world is R = Rz(yaw) * Ry(pitch) * Rx(roll).
/// Whether "world/body" means NED/FRD or ENU/FLU is stated by the owner of
/// the value; inside the legacy controller stack it is always NED/FRD.
struct EulerAnglesZyx {
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_COMMON_MATH_TYPES_HPP_
