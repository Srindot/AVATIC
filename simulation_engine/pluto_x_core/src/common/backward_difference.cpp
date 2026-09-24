// Copyright 2026 AVATIC contributors.

#include "pluto_x/common/backward_difference.hpp"

#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

Vector3 BackwardDifference3::Update(const Vector3& sample, double timestep_s) {
  if (!IsFinite(sample)) {
    throw std::invalid_argument("BackwardDifference3 sample must be finite");
  }
  if (!(timestep_s > 0.0) || !IsFinite(timestep_s)) {
    throw std::invalid_argument("BackwardDifference3 timestep must be > 0");
  }
  Vector3 derivative = Vector3::Zero();
  if (has_previous_) {
    derivative = (sample - previous_) / timestep_s;
  }
  previous_ = sample;
  has_previous_ = true;
  return derivative;
}

void BackwardDifference3::Reset() {
  previous_.setZero();
  has_previous_ = false;
}

}  // namespace pluto_x
