// Copyright 2026 AVATIC contributors.
//
// Balloon-popping score keeping, independent of the simulator.
//
// Rules implemented:
//  * a balloon pops the first time any of the vehicle's contact spheres
//    touches its surface (BalloonShape); it then no longer exists. The
//    spheres approximate the vehicle's outline (guards, body, camera) in
//    its body frame and move and rotate with it
//  * a popped balloon scores the points of its colour, once; points may be
//    negative (penalty balloons, e.g. red = -75), so the total can drop
//  * the run clock starts at clock_start_s (simulation start, or when the
//    vehicle arms, chosen by the caller); the run ends when
//    time - clock_start_s >= time_limit_s. Contacts at or after the end do
//    not score, so the result does not depend on how quickly the simulator
//    stops afterwards.
//
// Deterministic: Update() with the same inputs gives the same events.

#ifndef PLUTO_X_ARENA_ARENA_SCORING_HPP_
#define PLUTO_X_ARENA_ARENA_SCORING_HPP_

#include <optional>
#include <string>
#include <vector>

#include "pluto_x/arena/balloon_shape.hpp"
#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

struct BalloonSpec {
  std::string name;
  std::string color;
  int points{0};
  Vector3 centre_enu_m{Vector3::Zero()};
};

/// One sphere of the vehicle's contact outline, in the body frame of the
/// vehicle's reference link (Gazebo: FLU, origin at the CoM).
struct ContactSphere {
  Vector3 centre_body_m{Vector3::Zero()};
  double radius_m{0.0};
};

struct PopEvent {
  std::size_t index{0};  ///< into balloons()
  double time_s{0.0};    ///< simulation time of the contact
  double run_time_s{0.0};  ///< time since the run clock started
  int points{0};
  int total_points{0};
};

class ArenaScoring {
 public:
  /// Throws std::invalid_argument for a non-positive time limit, no contact
  /// spheres or a non-positive radius. Points may be negative (penalties).
  ArenaScoring(std::vector<BalloonSpec> balloons, BalloonShape shape,
               std::vector<ContactSphere> vehicle_contact_spheres,
               double time_limit_s);

  /// Starts the run clock (once; later calls are ignored).
  void StartClock(double time_s);
  bool clock_started() const { return clock_start_s_.has_value(); }

  /// Advances to time_s with the vehicle's reference point at vehicle_enu_m
  /// and attitude world_from_body. Returns the balloons popped in this call
  /// (in index order). No pops before the clock starts or after the run
  /// ended.
  std::vector<PopEvent> Update(double time_s, const Vector3& vehicle_enu_m,
                               const Matrix3& world_from_body = Matrix3::Identity());

  /// True once time - clock_start >= time_limit (latched).
  bool finished() const { return finished_; }
  /// Remaining run time (time limit before the clock starts; 0 when done).
  double remaining_s(double time_s) const;

  int total_points() const { return total_points_; }
  /// Best achievable score: the sum of the positive balloons (penalty
  /// balloons left alone).
  int max_points() const;
  const std::vector<BalloonSpec>& balloons() const { return balloons_; }
  bool popped(std::size_t index) const { return popped_.at(index); }
  std::size_t popped_count() const;
  const BalloonShape& shape() const { return shape_; }

 private:
  std::vector<BalloonSpec> balloons_;
  std::vector<bool> popped_;
  BalloonShape shape_;
  std::vector<ContactSphere> contact_spheres_;
  double time_limit_s_;
  std::optional<double> clock_start_s_;
  bool finished_{false};
  int total_points_{0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_ARENA_ARENA_SCORING_HPP_
