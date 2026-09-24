// Copyright 2026 AVATIC contributors.

#include "pluto_x/arena/balloon_shape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pluto_x {
namespace {

using Point2 = std::array<double, 2>;

double SegmentDistance(const Point2& p, const Point2& a, const Point2& b) {
  const double abx = b[0] - a[0];
  const double abz = b[1] - a[1];
  const double length2 = abx * abx + abz * abz;
  double t = 0.0;
  if (length2 > 0.0) {
    t = std::clamp(((p[0] - a[0]) * abx + (p[1] - a[1]) * abz) / length2, 0.0, 1.0);
  }
  return std::hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * abz));
}

}  // namespace

BalloonShape::BalloonShape(const std::vector<std::array<double, 2>>& profile_unit,
                           double diameter_m) {
  if (!(diameter_m > 0.0) || !std::isfinite(diameter_m)) {
    throw std::invalid_argument("balloon diameter must be > 0");
  }
  if (profile_unit.size() < 2) {
    throw std::invalid_argument("balloon profile needs at least 2 points");
  }
  for (std::size_t i = 0; i < profile_unit.size(); ++i) {
    const auto& p = profile_unit[i];
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || p[0] < 0.0 ||
        (i > 0 && p[1] < profile_unit[i - 1][1])) {
      throw std::invalid_argument(
          "balloon profile must have r >= 0 and non-decreasing z");
    }
    profile_.push_back({p[0] * diameter_m, p[1] * diameter_m});
    max_radius_m_ = std::max(max_radius_m_, p[0] * diameter_m);
  }
  if (!(profile_.back()[1] > profile_.front()[1])) {
    throw std::invalid_argument("balloon profile has zero height");
  }
}

bool BalloonShape::Contains(const Vector3& point_rel_m) const {
  const double z = point_rel_m.z();
  if (z < profile_.front()[1] || z > profile_.back()[1]) {
    return false;
  }
  const double rho = std::hypot(point_rel_m.x(), point_rel_m.y());
  for (std::size_t i = 1; i < profile_.size(); ++i) {
    const Point2& a = profile_[i - 1];
    const Point2& b = profile_[i];
    if (z <= b[1]) {
      const double dz = b[1] - a[1];
      const double r = dz > 0.0 ? a[0] + (b[0] - a[0]) * (z - a[1]) / dz
                                : std::max(a[0], b[0]);
      return rho <= r;
    }
  }
  return false;
}

double BalloonShape::DistanceToSurface(const Vector3& point_rel_m) const {
  if (Contains(point_rel_m)) {
    return 0.0;
  }
  const Point2 p{std::hypot(point_rel_m.x(), point_rel_m.y()), point_rel_m.z()};
  // closed boundary: bottom cap, profile, top cap
  double best = SegmentDistance(p, {0.0, profile_.front()[1]}, profile_.front());
  for (std::size_t i = 1; i < profile_.size(); ++i) {
    best = std::min(best, SegmentDistance(p, profile_[i - 1], profile_[i]));
  }
  best = std::min(best, SegmentDistance(p, profile_.back(), {0.0, profile_.back()[1]}));
  return best;
}

}  // namespace pluto_x
