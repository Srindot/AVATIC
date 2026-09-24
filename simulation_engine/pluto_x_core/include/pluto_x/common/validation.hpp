// Copyright 2026 AVATIC contributors.
//
// Small, explicit validation helpers.
//
// Policy:
//  * Configuration errors are reported once, at load time, via ConfigError
//    (see pluto_x/config/config_error.hpp).
//  * Violated preconditions inside algorithms are programming errors and
//    throw std::invalid_argument through RequireFinite()/RequireOrdered().
//  * Per-step runtime inputs (vehicle state, pilot input) are checked at the
//    entry point of each step and reported through a status value, never by
//    silently propagating NaN/Inf.

#ifndef PLUTO_X_COMMON_VALIDATION_HPP_
#define PLUTO_X_COMMON_VALIDATION_HPP_

#include <string>

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

/// True if value is neither NaN nor +/-Inf.
bool IsFinite(double value);

/// True if every component is finite.
bool IsFinite(const Vector3& value);

/// True if every angle is finite.
bool IsFinite(const EulerAnglesZyx& value);

/// Throws std::invalid_argument naming `what` if value is not finite.
void RequireFinite(double value, const std::string& what);

/// Throws std::invalid_argument naming `what` if lower > upper or either
/// bound is not finite.
void RequireOrdered(double lower, double upper, const std::string& what);

/// Clamps value into [lower, upper]. Precondition: lower <= upper (checked).
double Clamp(double value, double lower, double upper);

/// Clamps value into [-limit, +limit]. Precondition: limit >= 0 (checked).
double ClampSymmetric(double value, double limit);

}  // namespace pluto_x

#endif  // PLUTO_X_COMMON_VALIDATION_HPP_
