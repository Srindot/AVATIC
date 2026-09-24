// Copyright 2026 AVATIC contributors.

#include "pluto_x/common/validation.hpp"

#include <cmath>
#include <stdexcept>

namespace pluto_x {

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinite(const Vector3& value) { return value.allFinite(); }

bool IsFinite(const EulerAnglesZyx& value) {
  return IsFinite(value.roll_rad) && IsFinite(value.pitch_rad) &&
         IsFinite(value.yaw_rad);
}

void RequireFinite(double value, const std::string& what) {
  if (!IsFinite(value)) {
    throw std::invalid_argument(what + " must be finite");
  }
}

void RequireOrdered(double lower, double upper, const std::string& what) {
  RequireFinite(lower, what + " (lower bound)");
  RequireFinite(upper, what + " (upper bound)");
  if (lower > upper) {
    throw std::invalid_argument(what + ": lower bound exceeds upper bound");
  }
}

double Clamp(double value, double lower, double upper) {
  RequireOrdered(lower, upper, "Clamp bounds");
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

double ClampSymmetric(double value, double limit) {
  if (!(limit >= 0.0)) {
    throw std::invalid_argument("ClampSymmetric limit must be >= 0");
  }
  return Clamp(value, -limit, limit);
}

}  // namespace pluto_x
