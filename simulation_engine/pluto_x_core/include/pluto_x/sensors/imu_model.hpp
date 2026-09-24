// Copyright 2026 AVATIC contributors.
//
// Inertial measurement unit producing raw sensor counts, as the flight
// controller's gyro/accelerometer drivers would read them.
//
// Inputs (FRD body frame):
//   specific force  f = R^T (a - g)   (m/s^2; what an accelerometer measures:
//                                      +g "up" at rest, 0 in free fall)
//   angular rate    w                 (rad/s)
// Model per axis, then mapped into the sensor frame:
//   measured = true + bias + white noise
//   counts   = round(measured * scale), saturated at the full-scale range
//   sensor   = sensor_from_body * body   (signed axis permutation)
//
// Biases are drawn once per run (seeded); noise is white per sample.
// Values (ICM-20948 as configured by MagisV2): docs/pluto_x_parameters.md.

#ifndef PLUTO_X_SENSORS_IMU_MODEL_HPP_
#define PLUTO_X_SENSORS_IMU_MODEL_HPP_

#include <array>
#include <cstdint>
#include <random>

#include "pluto_x/common/math_types.hpp"

namespace pluto_x {

struct ImuParams {
  std::uint64_t seed{3};
  /// Counts per rad/s (ICM-20948 at +/-2000 dps: 16.4 LSB per deg/s).
  double gyro_counts_per_rad_s{0.0};
  /// Counts per m/s^2 (ICM-20948 at +/-8 g: 4096 LSB per g).
  double accel_counts_per_m_s2{0.0};
  double gyro_range_rad_s{0.0};
  double accel_range_m_s2{0.0};
  double gyro_noise_std_rad_s{0.0};
  double accel_noise_std_m_s2{0.0};
  double gyro_bias_std_rad_s{0.0};
  double accel_bias_std_m_s2{0.0};
  /// Signed axis permutation taking FRD body vectors to sensor axes.
  Matrix3 sensor_from_body{Matrix3::Identity()};
};

/// Throws std::invalid_argument naming the problem.
void ValidateImuParams(const ImuParams& params);

struct ImuRawSample {
  std::array<std::int16_t, 3> gyro_counts{};
  std::array<std::int16_t, 3> accel_counts{};
};

class ImuModel {
 public:
  explicit ImuModel(const ImuParams& params);

  /// Preconditions (checked): finite inputs.
  ImuRawSample Sample(const Vector3& specific_force_frd_m_s2,
                      const Vector3& angular_rate_frd_rad_s);

 private:
  std::array<std::int16_t, 3> Quantise(const Vector3& sensor_value,
                                       double counts_per_unit,
                                       double range) const;

  ImuParams params_;
  std::mt19937_64 engine_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  Vector3 gyro_bias_frd_{Vector3::Zero()};
  Vector3 accel_bias_frd_{Vector3::Zero()};
};

/// Specific force in FRD from world acceleration and attitude (NED world,
/// gravity along +z_NED with magnitude gravity_m_s2).
Vector3 SpecificForceFrd(const Vector3& acceleration_ned_m_s2,
                         const EulerAnglesZyx& attitude,
                         double gravity_m_s2);

}  // namespace pluto_x

#endif  // PLUTO_X_SENSORS_IMU_MODEL_HPP_
