// Copyright 2026 AVATIC contributors.
//
// One PID term with the exact semantics of the kwad.cpp loops:
//
//   if (|e| < integral_window)  I += e * dt
//   cp = kp * e
//   ci = clamp(ki * I, integral_term_min, integral_term_max)
//   cd = kd * derivative_signal          <-- measured signal, not de/dt
//   return cp + ci + cd                  <-- caller applies output mapping
//
// kwad.cpp stored sum(e) and multiplied by the fixed Ts; storing sum(e*dt)
// is identical for a fixed period and remains correct if dt ever varies.
// The accumulator itself is not clamped (legacy behaviour).

#ifndef PLUTO_X_CONTROL_LEGACY_PID_TERM_HPP_
#define PLUTO_X_CONTROL_LEGACY_PID_TERM_HPP_

#include "pluto_x/config/legacy_params.hpp"

namespace pluto_x {

class LegacyPidTerm {
 public:
  /// Throws std::invalid_argument if the integral bounds are unordered or
  /// any gain is not finite.
  LegacyPidTerm(const LegacyPidGains& gains, double integral_term_min,
                double integral_term_max);

  /// Advances the term by one step. Preconditions (checked, throw
  /// std::invalid_argument): all arguments finite, timestep_s > 0.
  double Update(double error, double derivative_signal, double timestep_s);

  /// Clears the integral accumulator.
  void Reset();

  /// Integral of error over time, in (error unit) * s.
  double integral_accumulator() const { return integral_accumulator_; }

 private:
  LegacyPidGains gains_;
  double integral_term_min_;
  double integral_term_max_;
  double integral_accumulator_{0.0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_LEGACY_PID_TERM_HPP_
