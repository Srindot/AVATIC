// Copyright 2026 AVATIC contributors.

#include "pluto_x/sensors/imu_model.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument("imu: " + message);
  }
}

bool IsSignedPermutation(const Matrix3& m) {
  for (int row = 0; row < 3; ++row) {
    int non_zero = 0;
    for (int col = 0; col < 3; ++col) {
      const double v = m(row, col);
      if (v == 1.0 || v == -1.0) {
        ++non_zero;
      } else if (v != 0.0) {
        return false;
      }
    }
    if (non_zero != 1) {
      return false;
    }
  }
  return std::abs(std::abs(m.determinant()) - 1.0) < 1e-12;
}

}  // namespace

void ValidateImuParams(const ImuParams& p) {
  Require(IsFinite(p.gyro_counts_per_rad_s) && p.gyro_counts_per_rad_s > 0.0,
          "gyro scale must be > 0");
  Require(IsFinite(p.accel_counts_per_m_s2) && p.accel_counts_per_m_s2 > 0.0,
          "accel scale must be > 0");
  Require(p.gyro_range_rad_s > 0.0 && p.accel_range_m_s2 > 0.0,
          "ranges must be > 0");
  Require(p.gyro_noise_std_rad_s >= 0.0 && p.accel_noise_std_m_s2 >= 0.0 &&
              p.gyro_bias_std_rad_s >= 0.0 && p.accel_bias_std_m_s2 >= 0.0,
          "noise/bias std must be >= 0");
  Require(IsSignedPermutation(p.sensor_from_body),
          "sensor_from_body must be a signed axis permutation");
}

ImuModel::ImuModel(const ImuParams& params)
    : params_(params), engine_(params.seed) {
  ValidateImuParams(params);
  for (int axis = 0; axis < 3; ++axis) {
    gyro_bias_frd_[axis] = params.gyro_bias_std_rad_s * unit_normal_(engine_);
    accel_bias_frd_[axis] = params.accel_bias_std_m_s2 * unit_normal_(engine_);
  }
}

ImuRawSample ImuModel::Sample(const Vector3& specific_force_frd_m_s2,
                              const Vector3& angular_rate_frd_rad_s) {
  if (!IsFinite(specific_force_frd_m_s2) || !IsFinite(angular_rate_frd_rad_s)) {
    throw std::invalid_argument("imu: inputs must be finite");
  }
  Vector3 gyro = angular_rate_frd_rad_s + gyro_bias_frd_;
  Vector3 accel = specific_force_frd_m_s2 + accel_bias_frd_;
  for (int axis = 0; axis < 3; ++axis) {
    gyro[axis] += params_.gyro_noise_std_rad_s * unit_normal_(engine_);
    accel[axis] += params_.accel_noise_std_m_s2 * unit_normal_(engine_);
  }
  ImuRawSample sample;
  sample.gyro_counts = Quantise(params_.sensor_from_body * gyro,
                                params_.gyro_counts_per_rad_s,
                                params_.gyro_range_rad_s);
  sample.accel_counts = Quantise(params_.sensor_from_body * accel,
                                 params_.accel_counts_per_m_s2,
                                 params_.accel_range_m_s2);
  return sample;
}

std::array<std::int16_t, 3> ImuModel::Quantise(const Vector3& value,
                                               double counts_per_unit,
                                               double range) const {
  constexpr double kInt16Max = std::numeric_limits<std::int16_t>::max();
  constexpr double kInt16Min = std::numeric_limits<std::int16_t>::min();
  const double full_scale = std::min(range * counts_per_unit, kInt16Max);
  std::array<std::int16_t, 3> counts{};
  for (int axis = 0; axis < 3; ++axis) {
    const double c = std::round(value[axis] * counts_per_unit);
    counts[axis] = static_cast<std::int16_t>(
        Clamp(c, std::max(-full_scale, kInt16Min), full_scale));
  }
  return counts;
}

Vector3 SpecificForceFrd(const Vector3& acceleration_ned_m_s2,
                         const EulerAnglesZyx& attitude,
                         double gravity_m_s2) {
  const Vector3 gravity_ned(0.0, 0.0, gravity_m_s2);
  return frames::RotationFromEulerZyx(attitude).transpose() *
         (acceleration_ned_m_s2 - gravity_ned);
}

}  // namespace pluto_x
