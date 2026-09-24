// Copyright 2026 AVATIC contributors.

#include "pluto_x/common/frames.hpp"

#include <algorithm>
#include <cmath>

namespace pluto_x {
namespace frames {
namespace {

/// Permutation matrix that swaps NED <-> ENU world axes. Symmetric and its
/// own inverse.
Matrix3 NedEnuSwapMatrix() {
  Matrix3 swap;
  swap << 0.0, 1.0, 0.0,
          1.0, 0.0, 0.0,
          0.0, 0.0, -1.0;
  return swap;
}

/// Reflection matrix that swaps FRD <-> FLU body axes. Diagonal and its own
/// inverse.
Matrix3 FrdFluSwapMatrix() {
  return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
}

}  // namespace

Vector3 SwapNedEnu(const Vector3& world_vector) {
  return Vector3(world_vector.y(), world_vector.x(), -world_vector.z());
}

Vector3 SwapFrdFlu(const Vector3& body_vector) {
  return Vector3(body_vector.x(), -body_vector.y(), -body_vector.z());
}

Matrix3 RotationNedFrdFromEnuFlu(const Matrix3& rotation_enu_flu) {
  // v_ned = S_w * v_enu,  v_flu = S_b * v_frd,  v_enu = R_enu_flu * v_flu
  // => v_ned = S_w * R_enu_flu * S_b * v_frd
  return NedEnuSwapMatrix() * rotation_enu_flu * FrdFluSwapMatrix();
}

Matrix3 RotationEnuFluFromNedFrd(const Matrix3& rotation_ned_frd) {
  // Both swap matrices are involutions, so the inverse has the same form.
  return NedEnuSwapMatrix() * rotation_ned_frd * FrdFluSwapMatrix();
}

Matrix3 RotationFromEulerZyx(const EulerAnglesZyx& euler) {
  const Eigen::AngleAxisd roll(euler.roll_rad, Vector3::UnitX());
  const Eigen::AngleAxisd pitch(euler.pitch_rad, Vector3::UnitY());
  const Eigen::AngleAxisd yaw(euler.yaw_rad, Vector3::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

EulerAnglesZyx EulerZyxFromRotation(const Matrix3& rotation_body_to_world) {
  const Matrix3& r = rotation_body_to_world;
  EulerAnglesZyx euler;
  // Clamp guards asin against |r(2,0)| marginally > 1 from rounding.
  euler.pitch_rad = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  euler.roll_rad = std::atan2(r(2, 1), r(2, 2));
  euler.yaw_rad = std::atan2(r(1, 0), r(0, 0));
  return euler;
}

Vector3 WorldToBodyZyx(const Vector3& world_vector,
                       const EulerAnglesZyx& euler) {
  return RotationFromEulerZyx(euler).transpose() * world_vector;
}

}  // namespace frames
}  // namespace pluto_x
