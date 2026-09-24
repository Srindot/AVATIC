// Copyright 2026 AVATIC contributors.
//
// Balloon surface as a surface of revolution about the world z axis
// (balloons are upright), for touch detection.
//
// The profile is a polyline of [r, z] points (bottom to top, relative to the
// balloon centre) extracted from the balloon mesh by
// pluto_x_gazebo/tools/prepare_balloon_mesh.py, for a unit diameter; it is
// scaled by the configured diameter. The solid is
//   { (rho, z) : z_first <= z <= z_last, rho <= r(z) }
// with r linearly interpolated, closed by flat caps at both ends.
//
// Because the shape is rotationally symmetric, the nearest surface point to
// any point P lies in P's meridian half-plane; the 3-D distance is therefore
// the 2-D distance from (rho, z) to the profile polyline (plus the caps),
// which is what DistanceToSurface computes.

#ifndef PLUTO_X_ARENA_BALLOON_SHAPE_HPP_
#define PLUTO_X_ARENA_BALLOON_SHAPE_HPP_

#include <array>
#include <vector>

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

class BalloonShape {
 public:
  /// profile_unit: [r, z] for a unit-diameter balloon, z non-decreasing,
  /// r >= 0, at least 2 points, nonzero height.
  /// Throws std::invalid_argument otherwise, or if diameter_m <= 0.
  BalloonShape(const std::vector<std::array<double, 2>>& profile_unit,
               double diameter_m);

  /// True if the point, relative to the balloon centre, is inside the solid.
  bool Contains(const Vector3& point_rel_m) const;

  /// Distance from the point (relative to the centre) to the surface; 0 if
  /// inside.
  double DistanceToSurface(const Vector3& point_rel_m) const;

  /// True if a sphere of the given radius at the point touches the balloon.
  bool TouchedBySphere(const Vector3& point_rel_m, double radius_m) const {
    return DistanceToSurface(point_rel_m) <= radius_m;
  }

  double height_m() const { return profile_.back()[1] - profile_.front()[1]; }
  double max_radius_m() const { return max_radius_m_; }

 private:
  std::vector<std::array<double, 2>> profile_;  // scaled [r, z]
  double max_radius_m_{0.0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_ARENA_BALLOON_SHAPE_HPP_
