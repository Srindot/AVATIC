// Copyright 2026 AVATIC contributors.

#include "pluto_x/environment/wind_model.hpp"

#include <cmath>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

WindModel::WindModel(const WindParams& params)
    : params_(params), engine_(params.seed) {
  if (!params.enabled) {
    return;
  }
  if (!IsFinite(params.mean_enu_m_s) || !IsFinite(params.gust_std_m_s) ||
      params.gust_std_m_s < 0.0 || !IsFinite(params.gust_time_constant_s) ||
      !(params.gust_time_constant_s > 0.0)) {
    throw std::invalid_argument(
        "wind: mean finite, gust std >= 0, gust time constant > 0 required");
  }
  velocity_enu_m_s_ = params.mean_enu_m_s;
}

Vector3 WindModel::Step(double timestep_s) {
  if (!IsFinite(timestep_s) || !(timestep_s > 0.0)) {
    throw std::invalid_argument("wind: timestep must be > 0");
  }
  if (!params_.enabled) {
    return velocity_enu_m_s_;
  }
  const double a = std::exp(-timestep_s / params_.gust_time_constant_s);
  const double innovation_std = params_.gust_std_m_s * std::sqrt(1.0 - a * a);
  for (int axis = 0; axis < 3; ++axis) {
    gust_enu_m_s_[axis] =
        a * gust_enu_m_s_[axis] + innovation_std * unit_normal_(engine_);
  }
  velocity_enu_m_s_ = params_.mean_enu_m_s + gust_enu_m_s_;
  return velocity_enu_m_s_;
}

}  // namespace pluto_x
