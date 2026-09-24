// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "pluto_x/sensors/state_error_model.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

TEST(StateErrorModel, DisabledIsIdentity) {
  StateErrorModel model(test::LoadDefaultConfig().state_error);
  test::RandomStates random(1);
  const VehicleStateNed state = random.State();
  const VehicleStateNed seen = model.Measure(state);
  EXPECT_EQ(seen.attitude.roll_rad, state.attitude.roll_rad);
  EXPECT_EQ(seen.body_rate_frd_rad_s, state.body_rate_frd_rad_s);
}

TEST(StateErrorModel, BiasIsConstantAndPositionUntouched) {
  StateErrorParams p = test::LoadPlutoXConfig().state_error;
  p.attitude_noise_std_rad = 0.0;
  p.gyro_noise_std_rad_s = 0.0;
  StateErrorModel model(p);
  VehicleStateNed state;
  state.position_ned_m = Vector3(1.0, 2.0, -3.0);
  const VehicleStateNed first = model.Measure(state);
  const VehicleStateNed second = model.Measure(state);
  EXPECT_EQ(first.attitude.roll_rad, second.attitude.roll_rad);
  EXPECT_EQ(first.attitude.roll_rad, model.attitude_bias_rad().roll_rad);
  EXPECT_NE(first.attitude.roll_rad, 0.0);
  EXPECT_EQ(first.attitude.yaw_rad, 0.0);  // no yaw bias
  EXPECT_EQ(first.body_rate_frd_rad_s, model.gyro_bias_rad_s());
  EXPECT_EQ(first.position_ned_m, state.position_ned_m);
}

TEST(StateErrorModel, DeterministicForSeedAndNoiseStatistics) {
  const StateErrorParams p = test::LoadPlutoXConfig().state_error;
  StateErrorModel a(p);
  StateErrorModel b(p);
  VehicleStateNed state;
  double sum_sq = 0.0;
  const int n = 100000;
  for (int i = 0; i < n; ++i) {
    const VehicleStateNed sa = a.Measure(state);
    const VehicleStateNed sb = b.Measure(state);
    ASSERT_EQ(sa.body_rate_frd_rad_s, sb.body_rate_frd_rad_s);
    const double noise = sa.body_rate_frd_rad_s.x() - a.gyro_bias_rad_s().x();
    sum_sq += noise * noise;
  }
  EXPECT_NEAR(std::sqrt(sum_sq / n), p.gyro_noise_std_rad_s,
              0.02 * p.gyro_noise_std_rad_s);
}

TEST(StateErrorModel, RejectsNegativeStd) {
  StateErrorParams p;
  p.enabled = true;
  p.gyro_noise_std_rad_s = -1.0;
  EXPECT_THROW(StateErrorModel{p}, std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
