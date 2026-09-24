// Copyright 2026 AVATIC contributors.
//
// Corrupts the true vehicle state into the state the controller sees.
//
// PLACEHOLDER FOR A REAL SENSOR + ESTIMATOR CHAIN. The legacy controller
// consumes a full state, so this model perturbs that state directly:
//   attitude_seen = attitude + b_att + n_att     (b_att on roll/pitch only)
//   rate_seen     = rate     + b_gyro + n_gyro
//   position, velocity: unchanged (legacy position hold uses ground truth)
// b_* are drawn once per run; n_* are white per call. The roll/pitch bias
// reproduces the "level trim" error that makes a real small quadrotor drift
// in angle mode. Superseded once MagisV2's own estimator runs on simulated
// IMU samples. Deterministic for a given seed and call sequence.

#ifndef PLUTO_X_SENSORS_STATE_ERROR_MODEL_HPP_
#define PLUTO_X_SENSORS_STATE_ERROR_MODEL_HPP_

#include <random>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

class StateErrorModel {
 public:
  /// Throws std::invalid_argument for negative or non-finite std values.
  explicit StateErrorModel(const StateErrorParams& params);

  /// Returns the perturbed state (identity if disabled).
  VehicleStateNed Measure(const VehicleStateNed& true_state);

  const EulerAnglesZyx& attitude_bias_rad() const { return attitude_bias_; }
  const Vector3& gyro_bias_rad_s() const { return gyro_bias_; }

 private:
  double Normal(double std_dev);

  StateErrorParams params_;
  std::mt19937_64 engine_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  EulerAnglesZyx attitude_bias_;
  Vector3 gyro_bias_{Vector3::Zero()};
};

}  // namespace pluto_x

#endif  // PLUTO_X_SENSORS_STATE_ERROR_MODEL_HPP_
