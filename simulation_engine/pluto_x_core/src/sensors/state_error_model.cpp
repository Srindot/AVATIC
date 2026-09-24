// Copyright 2026 AVATIC contributors.

#include "pluto_x/sensors/state_error_model.hpp"

#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

bool IsValidStd(double value) { return IsFinite(value) && value >= 0.0; }

}  // namespace

StateErrorModel::StateErrorModel(const StateErrorParams& params)
    : params_(params), engine_(params.seed) {
  if (!IsValidStd(params.attitude_bias_std_rad) ||
      !IsValidStd(params.attitude_noise_std_rad) ||
      !IsValidStd(params.gyro_bias_std_rad_s) ||
      !IsValidStd(params.gyro_noise_std_rad_s)) {
    throw std::invalid_argument("state error std values must be >= 0");
  }
  if (!params.enabled) {
    return;
  }
  attitude_bias_.roll_rad = Normal(params.attitude_bias_std_rad);
  attitude_bias_.pitch_rad = Normal(params.attitude_bias_std_rad);
  for (int axis = 0; axis < 3; ++axis) {
    gyro_bias_[axis] = Normal(params.gyro_bias_std_rad_s);
  }
}

VehicleStateNed StateErrorModel::Measure(const VehicleStateNed& true_state) {
  if (!params_.enabled) {
    return true_state;
  }
  VehicleStateNed seen = true_state;
  const double n_att = params_.attitude_noise_std_rad;
  seen.attitude.roll_rad += attitude_bias_.roll_rad + Normal(n_att);
  seen.attitude.pitch_rad += attitude_bias_.pitch_rad + Normal(n_att);
  seen.attitude.yaw_rad += Normal(n_att);
  for (int axis = 0; axis < 3; ++axis) {
    seen.body_rate_frd_rad_s[axis] +=
        gyro_bias_[axis] + Normal(params_.gyro_noise_std_rad_s);
  }
  return seen;
}

double StateErrorModel::Normal(double std_dev) {
  return std_dev * unit_normal_(engine_);
}

}  // namespace pluto_x
