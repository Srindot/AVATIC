// Copyright 2026 AVATIC contributors.

#include "pluto_x/arena/arena_scoring.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pluto_x {

ArenaScoring::ArenaScoring(std::vector<BalloonSpec> balloons, BalloonShape shape,
                           std::vector<ContactSphere> vehicle_contact_spheres,
                           double time_limit_s)
    : balloons_(std::move(balloons)),
      popped_(balloons_.size(), false),
      shape_(std::move(shape)),
      contact_spheres_(std::move(vehicle_contact_spheres)),
      time_limit_s_(time_limit_s) {
  if (contact_spheres_.empty()) {
    throw std::invalid_argument("at least one vehicle contact sphere is required");
  }
  for (const ContactSphere& c : contact_spheres_) {
    if (!(c.radius_m > 0.0) || !std::isfinite(c.radius_m) ||
        !c.centre_body_m.allFinite()) {
      throw std::invalid_argument("vehicle contact spheres need finite centres, radius > 0");
    }
  }
  if (!(time_limit_s > 0.0) || !std::isfinite(time_limit_s)) {
    throw std::invalid_argument("time limit must be > 0");
  }

}

void ArenaScoring::StartClock(double time_s) {
  if (!clock_start_s_) {
    clock_start_s_ = time_s;
  }
}

std::vector<PopEvent> ArenaScoring::Update(double time_s,
                                           const Vector3& vehicle_enu_m,
                                           const Matrix3& world_from_body) {
  std::vector<PopEvent> events;
  if (!clock_start_s_ || finished_) {
    return events;
  }
  const double run_time_s = time_s - *clock_start_s_;
  if (run_time_s >= time_limit_s_) {
    finished_ = true;
    return events;
  }
  for (std::size_t i = 0; i < balloons_.size(); ++i) {
    if (popped_[i]) {
      continue;
    }
    bool touched = false;
    for (const ContactSphere& c : contact_spheres_) {
      const Vector3 centre = vehicle_enu_m + world_from_body * c.centre_body_m;
      if (shape_.TouchedBySphere(centre - balloons_[i].centre_enu_m, c.radius_m)) {
        touched = true;
        break;
      }
    }
    if (touched) {
      popped_[i] = true;
      total_points_ += balloons_[i].points;
      events.push_back({i, time_s, run_time_s, balloons_[i].points, total_points_});
    }
  }
  return events;
}

double ArenaScoring::remaining_s(double time_s) const {
  if (!clock_start_s_) {
    return time_limit_s_;
  }
  if (finished_) {
    return 0.0;
  }
  return std::max(time_limit_s_ - (time_s - *clock_start_s_), 0.0);
}

int ArenaScoring::max_points() const {
  int sum = 0;
  for (const BalloonSpec& b : balloons_) sum += std::max(b.points, 0);
  return sum;
}

std::size_t ArenaScoring::popped_count() const {
  return static_cast<std::size_t>(std::count(popped_.begin(), popped_.end(), true));
}

}  // namespace pluto_x
