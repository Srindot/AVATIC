// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/enu_adapter.hpp"

#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

/// Quaternion norms below this are treated as degenerate. Chosen far below
/// any value a physics engine produces for a unit quaternion and far above
/// double-precision noise.
constexpr double kMinimumQuaternionNorm = 1e-6;

Matrix3 RotationFromQuaternion(const Eigen::Quaterniond& orientation) {
  if (!orientation.coeffs().allFinite() ||
      orientation.norm() < kMinimumQuaternionNorm) {
    throw std::invalid_argument("orientation quaternion is degenerate");
  }
  return orientation.normalized().toRotationMatrix();
}

}  // namespace

bool IsFinite(const RigidBodyStateEnu& state) {
  return IsFinite(state.position_enu_m) &&
         state.orientation_enu_flu.coeffs().allFinite() &&
         IsFinite(state.velocity_enu_m_s) &&
         IsFinite(state.angular_velocity_enu_rad_s);
}

VehicleStateNed ToVehicleStateNed(const RigidBodyStateEnu& state) {
  if (!IsFinite(state)) {
    throw std::invalid_argument("simulator state must be finite");
  }
  const Matrix3 rotation_enu_flu =
      RotationFromQuaternion(state.orientation_enu_flu);

  VehicleStateNed legacy;
  legacy.position_ned_m = frames::SwapNedEnu(state.position_enu_m);
  legacy.velocity_ned_m_s = frames::SwapNedEnu(state.velocity_enu_m_s);
  legacy.attitude = frames::EulerZyxFromRotation(
      frames::RotationNedFrdFromEnuFlu(rotation_enu_flu));
  const Vector3 body_rate_flu =
      rotation_enu_flu.transpose() * state.angular_velocity_enu_rad_s;
  legacy.body_rate_frd_rad_s = frames::SwapFrdFlu(body_rate_flu);
  return legacy;
}

WrenchEnuWorld ToWrenchEnuWorld(const ExternalWrenchNedFrd& wrench,
                                const Eigen::Quaterniond& orientation_enu_flu) {
  if (!IsFinite(wrench.force_ned_n) || !IsFinite(wrench.torque_frd_n_m)) {
    throw std::invalid_argument("wrench must be finite");
  }
  const Matrix3 rotation_enu_flu = RotationFromQuaternion(orientation_enu_flu);
  WrenchEnuWorld out;
  out.force_enu_n = frames::SwapNedEnu(wrench.force_ned_n);
  out.torque_enu_n_m =
      rotation_enu_flu * frames::SwapFrdFlu(wrench.torque_frd_n_m);
  return out;
}

}  // namespace pluto_x
