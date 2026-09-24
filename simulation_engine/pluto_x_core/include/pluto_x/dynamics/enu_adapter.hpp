// Copyright 2026 AVATIC contributors.
//
// Conversion between a simulator rigid-body state expressed the ROS/Gazebo
// way (ENU world, FLU body) and the NED/FRD quantities used by the legacy
// controller and dynamics. Pure mathematics: no Gazebo types, so the plugin
// only has to copy gz::math values into these structures.

#ifndef PLUTO_X_DYNAMICS_ENU_ADAPTER_HPP_
#define PLUTO_X_DYNAMICS_ENU_ADAPTER_HPP_

#include <Eigen/Geometry>

#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

/// Rigid-body state of the vehicle's centre-of-mass link as reported by the
/// simulator.
struct RigidBodyStateEnu {
  Vector3 position_enu_m{Vector3::Zero()};
  /// Rotation of the FLU body frame relative to the ENU world.
  Eigen::Quaterniond orientation_enu_flu{Eigen::Quaterniond::Identity()};
  Vector3 velocity_enu_m_s{Vector3::Zero()};
  /// Angular velocity expressed in the ENU WORLD frame (Gazebo convention).
  Vector3 angular_velocity_enu_rad_s{Vector3::Zero()};
};

bool IsFinite(const RigidBodyStateEnu& state);

/// ENU/FLU simulator state -> NED/FRD legacy state.
/// Precondition (checked, throws std::invalid_argument): finite state with a
/// non-degenerate quaternion. The quaternion is normalised internally.
VehicleStateNed ToVehicleStateNed(const RigidBodyStateEnu& state);

/// Wrench ready to be applied to the simulator link: force and torque both
/// expressed in the ENU WORLD frame, acting at the centre of mass.
struct WrenchEnuWorld {
  Vector3 force_enu_n{Vector3::Zero()};
  Vector3 torque_enu_n_m{Vector3::Zero()};
};

/// NED-force / FRD-torque wrench -> ENU-world wrench for the given attitude.
/// Precondition (checked): finite inputs, non-degenerate quaternion.
WrenchEnuWorld ToWrenchEnuWorld(const ExternalWrenchNedFrd& wrench,
                                const Eigen::Quaterniond& orientation_enu_flu);

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_ENU_ADAPTER_HPP_
