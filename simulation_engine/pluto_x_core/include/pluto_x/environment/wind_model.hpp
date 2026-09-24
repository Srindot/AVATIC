// Copyright 2026 AVATIC contributors.
//
// Wind velocity: configured mean plus an independent first-order
// Gauss-Markov (Ornstein-Uhlenbeck) gust on each axis, updated with the exact
// discretisation
//
//   g <- a g + sigma sqrt(1 - a^2) n,   a = exp(-dt / tau),  n ~ N(0, 1)
//
// so the gust's stationary standard deviation is sigma regardless of dt.
// ASSUMPTION: spatially uniform wind; it acts through the air-relative
// velocity in the linear drag term only (see ComputeLegacyExternalWrench).
// Deterministic for a given seed and step sequence.

#ifndef PLUTO_X_ENVIRONMENT_WIND_MODEL_HPP_
#define PLUTO_X_ENVIRONMENT_WIND_MODEL_HPP_

#include <random>

#include "pluto_x/config/legacy_params.hpp"

namespace pluto_x {

class WindModel {
 public:
  /// Throws std::invalid_argument for invalid parameters (enabled only).
  explicit WindModel(const WindParams& params);

  /// Advances the gust by timestep_s (> 0, checked) and returns the wind
  /// velocity in ENU. Returns zero if disabled.
  Vector3 Step(double timestep_s);

  const Vector3& velocity_enu_m_s() const { return velocity_enu_m_s_; }

 private:
  WindParams params_;
  std::mt19937_64 engine_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  Vector3 gust_enu_m_s_{Vector3::Zero()};
  Vector3 velocity_enu_m_s_{Vector3::Zero()};
};

}  // namespace pluto_x

#endif  // PLUTO_X_ENVIRONMENT_WIND_MODEL_HPP_
