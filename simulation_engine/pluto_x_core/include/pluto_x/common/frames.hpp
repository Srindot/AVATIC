// Copyright 2026 AVATIC contributors.
//
// Coordinate-frame conventions and conversions.
//
// Frames used in this project:
//
//   ENU  world, x = East,  y = North, z = Up     (ROS REP-103, Gazebo world)
//   FLU  body,  x = Forward, y = Left, z = Up    (ROS REP-103, Gazebo link)
//   NED  world, x = North, y = East,  z = Down   (legacy controller stack)
//   FRD  body,  x = Forward, y = Right, z = Down (legacy controller stack)
//
// The legacy kwad.cpp equations are written in NED/FRD (thrust acts along
// -z_body, gravity along +z_world). All legacy mathematics therefore runs in
// NED/FRD and every conversion to/from Gazebo happens through this module.
//
// Both mappings used here are involutions (applying them twice is identity):
//   NED <-> ENU : (a, b, c) -> (b, a, -c)
//   FRD <-> FLU : (a, b, c) -> (a, -b, -c)

#ifndef PLUTO_X_COMMON_FRAMES_HPP_
#define PLUTO_X_COMMON_FRAMES_HPP_

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {
namespace frames {

/// Re-expresses a world-frame vector ENU -> NED (or NED -> ENU).
Vector3 SwapNedEnu(const Vector3& world_vector);

/// Re-expresses a body-frame vector FLU -> FRD (or FRD -> FLU).
Vector3 SwapFrdFlu(const Vector3& body_vector);

/// Given R_enu_flu (maps FLU body vectors into ENU world), returns R_ned_frd
/// (maps FRD body vectors into NED world) describing the same physical
/// attitude.
Matrix3 RotationNedFrdFromEnuFlu(const Matrix3& rotation_enu_flu);

/// Inverse of RotationNedFrdFromEnuFlu.
Matrix3 RotationEnuFluFromNedFrd(const Matrix3& rotation_ned_frd);

/// Body-to-world rotation for Z-Y-X Euler angles: Rz(yaw) Ry(pitch) Rx(roll).
Matrix3 RotationFromEulerZyx(const EulerAnglesZyx& euler);

/// Extracts Z-Y-X Euler angles from a body-to-world rotation matrix.
/// pitch is returned in [-pi/2, pi/2]; roll and yaw in (-pi, pi].
/// Near pitch = +/-pi/2 (gimbal lock) roll/yaw are not unique; the result is
/// still a valid decomposition.
EulerAnglesZyx EulerZyxFromRotation(const Matrix3& rotation_body_to_world);

/// Expresses a world-frame vector in the body frame given Z-Y-X Euler angles
/// (R^T * v). This is exactly the legacy kwad.cpp rotateGFtoBF() function,
/// written out term by term there.
Vector3 WorldToBodyZyx(const Vector3& world_vector,
                       const EulerAnglesZyx& euler);

}  // namespace frames
}  // namespace pluto_x

#endif  // PLUTO_X_COMMON_FRAMES_HPP_
