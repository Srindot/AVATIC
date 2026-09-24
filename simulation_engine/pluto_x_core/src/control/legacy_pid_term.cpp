// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/legacy_pid_term.hpp"

#include <cmath>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

LegacyPidTerm::LegacyPidTerm(const LegacyPidGains& gains,
                             double integral_term_min,
                             double integral_term_max)
    : gains_(gains),
      integral_term_min_(integral_term_min),
      integral_term_max_(integral_term_max) {
  RequireFinite(gains.kp, "LegacyPidTerm kp");
  RequireFinite(gains.ki, "LegacyPidTerm ki");
  RequireFinite(gains.kd, "LegacyPidTerm kd");
  RequireFinite(gains.integral_window, "LegacyPidTerm integral_window");
  RequireOrdered(integral_term_min, integral_term_max,
                 "LegacyPidTerm integral term bounds");
}

double LegacyPidTerm::Update(double error, double derivative_signal,
                             double timestep_s) {
  RequireFinite(error, "LegacyPidTerm error");
  RequireFinite(derivative_signal, "LegacyPidTerm derivative signal");
  if (!(timestep_s > 0.0) || !IsFinite(timestep_s)) {
    throw std::invalid_argument("LegacyPidTerm timestep must be > 0");
  }

  if (std::abs(error) < gains_.integral_window) {
    integral_accumulator_ += error * timestep_s;
  }
  const double proportional_term = gains_.kp * error;
  const double integral_term =
      Clamp(gains_.ki * integral_accumulator_, integral_term_min_,
            integral_term_max_);
  const double derivative_term = gains_.kd * derivative_signal;
  return proportional_term + integral_term + derivative_term;
}

void LegacyPidTerm::Reset() { integral_accumulator_ = 0.0; }

}  // namespace pluto_x
