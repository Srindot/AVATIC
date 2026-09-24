// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "pluto_x/sensors/baro_model.hpp"
#include "pluto_x/sensors/imu_model.hpp"

namespace pluto_x {
namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kG = 9.80665;

ImuParams IdealIcm20948() {
  ImuParams p;
  p.gyro_counts_per_rad_s = 16.4 / kDegToRad;   // 16.4 LSB per deg/s
  p.accel_counts_per_m_s2 = 4096.0 / kG;        // 4096 LSB per g
  p.gyro_range_rad_s = 2000.0 * kDegToRad;
  p.accel_range_m_s2 = 8.0 * kG;
  return p;
}

TEST(ImuModel, LevelAtRestReadsOneGUpward) {
  ImuModel imu(IdealIcm20948());
  const Vector3 f = SpecificForceFrd(Vector3::Zero(), {}, kG);
  EXPECT_NEAR(f.z(), -kG, 1e-12);  // FRD z points down: specific force -g
  const ImuRawSample s = imu.Sample(f, Vector3::Zero());
  EXPECT_EQ(s.accel_counts[0], 0);
  EXPECT_EQ(s.accel_counts[1], 0);
  EXPECT_EQ(s.accel_counts[2], -4096);
  EXPECT_EQ(s.gyro_counts[2], 0);
}

TEST(ImuModel, FreeFallReadsZero) {
  const Vector3 f =
      SpecificForceFrd(Vector3(0.0, 0.0, kG), {0.3, -0.2, 1.0}, kG);
  EXPECT_LT(f.norm(), 1e-12);
}

TEST(ImuModel, ScaleQuantisationAndSaturation) {
  ImuModel imu(IdealIcm20948());
  ImuRawSample s = imu.Sample(Vector3::Zero(),
                              Vector3(100.0 * kDegToRad, 0.0, 0.0));
  EXPECT_EQ(s.gyro_counts[0], 1640);  // 100 deg/s * 16.4
  s = imu.Sample(Vector3(0.0, 0.0, 20.0 * kG), Vector3(0.0, 0.0, 50.0));
  EXPECT_EQ(s.accel_counts[2], 32767);  // clamped at +/-8 g = 32768 counts
  EXPECT_EQ(s.gyro_counts[2], 32767);   // clamped at +/-2000 dps
}

TEST(ImuModel, SensorAxisPermutationIsApplied) {
  ImuParams p = IdealIcm20948();
  p.sensor_from_body << 0, 1, 0,   // sensor x = body y
                        1, 0, 0,   // sensor y = body x
                        0, 0, -1;  // sensor z = -body z
  ImuModel imu(p);
  const ImuRawSample s = imu.Sample(Vector3(0.0, 0.0, -kG),
                                    Vector3(0.1, 0.0, 0.0));
  EXPECT_EQ(s.accel_counts[2], 4096);
  EXPECT_GT(s.gyro_counts[1], 0);
  EXPECT_EQ(s.gyro_counts[0], 0);
  p.sensor_from_body(0, 0) = 0.5;
  EXPECT_THROW(ImuModel{p}, std::invalid_argument);
}

TEST(ImuModel, SeededNoiseIsDeterministic) {
  ImuParams p = IdealIcm20948();
  p.gyro_noise_std_rad_s = 0.01;
  p.accel_bias_std_m_s2 = 0.05;
  ImuModel a(p);
  ImuModel b(p);
  for (int i = 0; i < 100; ++i) {
    const auto sa = a.Sample(Vector3(0, 0, -kG), Vector3::Zero());
    const auto sb = b.Sample(Vector3(0, 0, -kG), Vector3::Zero());
    ASSERT_EQ(sa.gyro_counts, sb.gyro_counts);
    ASSERT_EQ(sa.accel_counts, sb.accel_counts);
  }
}

TEST(BaroModel, IsaReferencePoints) {
  EXPECT_NEAR(IsaPressurePa(0.0, 101325.0), 101325.0, 1e-9);
  EXPECT_NEAR(IsaPressurePa(1000.0, 101325.0), 89874.6, 1.0);  // ISA table
  // ~12 Pa per metre near sea level.
  const double dp = IsaPressurePa(0.0, 101325.0) - IsaPressurePa(1.0, 101325.0);
  EXPECT_NEAR(dp, 12.0, 0.2);
}

TEST(BaroModel, HeightAboveGroundAltitude) {
  BaroParams p;
  p.ground_altitude_msl_m = 500.0;
  BaroModel baro(p);
  EXPECT_NEAR(baro.Sample(0.0).pressure_pa, IsaPressurePa(500.0, 101325.0),
              1e-9);
  // Hydrostatic gradient rho g = p M g / (R T) at 95.46 kPa, 25 degC
  // (default): 10.94 Pa per metre.
  EXPECT_NEAR(baro.PressureAtHeightPa(0.0) - baro.PressureAtHeightPa(1.0),
              10.94, 0.02);
  p.noise_std_pa = -1.0;
  EXPECT_THROW(BaroModel{p}, std::invalid_argument);
}

// The pressure-height gradient follows the configured air temperature, so
// the hypsometric formula MagisV2 uses (isothermal at the sensor
// temperature, R/(M g) = 29.271267 m/K) recovers the height.
TEST(BaroModel, ConsistentWithSensorTemperature) {
  BaroParams p;
  p.ground_altitude_msl_m = 505.0;
  p.temperature_c = 30.0;
  const BaroModel baro(p);
  const double h = 20.0;
  const double recovered =
      std::log(baro.PressureAtHeightPa(0.0) / baro.PressureAtHeightPa(h)) *
      (p.temperature_c + 273.15) * 29.271267;
  EXPECT_NEAR(recovered, h, 0.01 * h);
  p.temperature_c = 80.0;
  EXPECT_THROW(BaroModel{p}, std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
