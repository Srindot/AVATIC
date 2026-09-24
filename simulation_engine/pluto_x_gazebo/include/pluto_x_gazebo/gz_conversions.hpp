// Copyright 2026 AVATIC contributors.
//
// Value conversions between gz::math and the Eigen types used by
// pluto_x_core. No frame changes happen here: both sides are ENU world.

#ifndef PLUTO_X_GAZEBO_GZ_CONVERSIONS_HPP_
#define PLUTO_X_GAZEBO_GZ_CONVERSIONS_HPP_

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "pluto_x/common/math_types.hpp"

namespace pluto_x_gazebo {

inline pluto_x::Vector3 ToEigen(const gz::math::Vector3d& v) {
  return pluto_x::Vector3(v.X(), v.Y(), v.Z());
}

inline Eigen::Quaterniond ToEigen(const gz::math::Quaterniond& q) {
  return Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z());
}

inline gz::math::Vector3d ToGz(const pluto_x::Vector3& v) {
  return gz::math::Vector3d(v.x(), v.y(), v.z());
}

}  // namespace pluto_x_gazebo

#endif  // PLUTO_X_GAZEBO_GZ_CONVERSIONS_HPP_
