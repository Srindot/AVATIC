// Copyright 2026 AVATIC contributors.
//
// First-order backward-difference differentiator for a vector signal.
//
// Used to obtain the body angular acceleration required by the legacy rate
// loop's derivative term when the plant is Gazebo (kwad.cpp read it directly
// from its own integrator). The first sample after construction or Reset()
// yields zero.

#ifndef PLUTO_X_COMMON_BACKWARD_DIFFERENCE_HPP_
#define PLUTO_X_COMMON_BACKWARD_DIFFERENCE_HPP_

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

class BackwardDifference3 {
 public:
  /// Returns (sample - previous) / timestep_s, or zero on the first call.
  /// Preconditions (checked, throw std::invalid_argument): finite sample,
  /// timestep_s > 0.
  Vector3 Update(const Vector3& sample, double timestep_s);

  void Reset();

 private:
  Vector3 previous_{Vector3::Zero()};
  bool has_previous_{false};
};

}  // namespace pluto_x

#endif  // PLUTO_X_COMMON_BACKWARD_DIFFERENCE_HPP_
