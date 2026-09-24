// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluto_x/control/quad_mixer.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

/// kwad.cpp quad_motor_speed() allocation, verbatim (before clamping).
std::array<double, 4> LegacyAllocation(const BodyWrenchCommand& c, double kt,
                                       double kd, double l) {
  const double u1 = c.thrust_n;
  const double u2 = c.torque_frd_n_m.x();
  const double u3 = c.torque_frd_n_m.y();
  const double u4 = c.torque_frd_n_m.z();
  return {u1 / (4 * kt) + u3 / (2 * kt * l) + u4 / (4 * kd),
          u1 / (4 * kt) - u2 / (2 * kt * l) - u4 / (4 * kd),
          u1 / (4 * kt) - u3 / (2 * kt * l) + u4 / (4 * kd),
          u1 / (4 * kt) + u2 / (2 * kt * l) - u4 / (4 * kd)};
}

class LegacyPlusMixerTest : public ::testing::Test {
 protected:
  LegacyStackConfig config_ = test::LoadDefaultConfig();
  QuadMixer mixer_{config_.rotor, config_.vehicle.arm_length_m};
};

TEST_F(LegacyPlusMixerTest, HoverThrustSplitsEvenly) {
  BodyWrenchCommand command;
  command.thrust_n = 13.734;
  const MixerOutput out = mixer_.Mix(command);
  // w^2 = u1 / (4 kt) = 13.734 / (4 * 1.3328e-5)
  const double expected_w2 = 13.734 / (4.0 * 1.3328e-5);
  for (int i = 0; i < kRotorCount; ++i) {
    EXPECT_NEAR(out.rotor_speed_squared_rad2_s2[i], expected_w2, 1e-6);
    EXPECT_NEAR(out.rotor_speed_rad_s[i], std::sqrt(expected_w2), 1e-9);
  }
  EXPECT_NEAR(out.achieved.thrust_n, 13.734, 1e-9);
  EXPECT_NEAR(out.achieved.torque_frd_n_m.norm(), 0.0, 1e-9);
  EXPECT_NEAR(out.net_rotor_speed_rad_s, 0.0, 1e-9);
  EXPECT_FALSE(out.saturated);
}

TEST_F(LegacyPlusMixerTest, MatchesKwadCppAllocationExactly) {
  test::RandomStates random(5);
  for (int i = 0; i < 200; ++i) {
    BodyWrenchCommand command;
    command.thrust_n = random.Uniform(10.0, 20.0);
    command.torque_frd_n_m = Vector3(random.Uniform(-1.0, 1.0),
                                     random.Uniform(-1.0, 1.0),
                                     random.Uniform(-0.05, 0.05));
    const MixerOutput out = mixer_.Mix(command);
    const auto legacy = LegacyAllocation(
        command, config_.rotor.thrust_coefficient_n_s2,
        config_.rotor.yaw_drag_coefficient_n_m_s2,
        config_.vehicle.arm_length_m);
    for (int r = 0; r < kRotorCount; ++r) {
      EXPECT_NEAR(out.rotor_speed_squared_rad2_s2[r], legacy[r],
                  1e-9 * legacy[r]);
    }
    ASSERT_FALSE(out.saturated);
    EXPECT_NEAR(out.achieved.thrust_n, command.thrust_n, 1e-9);
    EXPECT_LT((out.achieved.torque_frd_n_m - command.torque_frd_n_m).norm(),
              1e-9);
    // Legacy o = w1 - w2 + w3 - w4
    const auto& w = out.rotor_speed_rad_s;
    EXPECT_NEAR(out.net_rotor_speed_rad_s, w[0] - w[1] + w[2] - w[3], 1e-9);
  }
}

TEST_F(LegacyPlusMixerTest, SaturatesAtRotorLimits) {
  BodyWrenchCommand command;
  command.thrust_n = 1000.0;
  MixerOutput out = mixer_.Mix(command);
  EXPECT_TRUE(out.saturated);
  for (double w2 : out.rotor_speed_squared_rad2_s2) {
    EXPECT_DOUBLE_EQ(w2, config_.rotor.speed_squared_max_rad2_s2);
  }
  EXPECT_NEAR(out.achieved.thrust_n,
              4.0 * config_.rotor.thrust_coefficient_n_s2 *
                  config_.rotor.speed_squared_max_rad2_s2,
              1e-9);

  command.thrust_n = 0.0;
  command.torque_frd_n_m = Vector3(1.0, 0.0, 0.0);
  out = mixer_.Mix(command);
  EXPECT_TRUE(out.saturated);
  EXPECT_DOUBLE_EQ(out.rotor_speed_squared_rad2_s2[1], 0.0);  // no negatives
}

TEST_F(LegacyPlusMixerTest, RejectsInvalidInputAndParameters) {
  BodyWrenchCommand command;
  command.thrust_n = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(mixer_.Mix(command), std::invalid_argument);

  LegacyRotorParams bad = config_.rotor;
  bad.thrust_coefficient_n_s2 = 0.0;
  EXPECT_THROW(QuadMixer(bad, 0.5), std::invalid_argument);
  EXPECT_THROW(QuadMixer(config_.rotor, -0.5), std::invalid_argument);
}

class QuadXMixerTest : public ::testing::Test {
 protected:
  LegacyStackConfig config_ = test::LoadPlutoXConfig();
  QuadMixer mixer_{config_.rotor, config_.vehicle.arm_length_m};
  double hover_n_ = config_.vehicle.mass_kg * config_.vehicle.gravity_m_s2;

  MixerOutput MixTorque(const Vector3& torque) const {
    BodyWrenchCommand command;
    command.thrust_n = hover_n_;
    command.torque_frd_n_m = torque;
    return mixer_.Mix(command);
  }
};

// MagisV2 order: M0 rear-right, M1 front-right, M2 rear-left, M3 front-left.
TEST_F(QuadXMixerTest, GeometryMatchesMagisV2Order) {
  const auto& g = mixer_.geometry();
  const double c = config_.vehicle.arm_length_m / std::sqrt(2.0);
  EXPECT_NEAR(g[0].x_frd_m, -c, 1e-12);  // rear
  EXPECT_NEAR(g[0].y_frd_m, c, 1e-12);   // right
  EXPECT_NEAR(g[1].x_frd_m, c, 1e-12);   // front
  EXPECT_NEAR(g[1].y_frd_m, c, 1e-12);   // right
  EXPECT_NEAR(g[2].x_frd_m, -c, 1e-12);  // rear
  EXPECT_NEAR(g[2].y_frd_m, -c, 1e-12);  // left
  EXPECT_NEAR(g[3].x_frd_m, c, 1e-12);   // front
  EXPECT_NEAR(g[3].y_frd_m, -c, 1e-12);  // left
}

TEST_F(QuadXMixerTest, TorqueDirectionsSpeedUpTheRightMotors) {
  // Roll right (right side down): left motors M2, M3 faster.
  MixerOutput out = MixTorque(Vector3(0.002, 0.0, 0.0));
  EXPECT_GT(out.rotor_speed_rad_s[2], out.rotor_speed_rad_s[0]);
  EXPECT_GT(out.rotor_speed_rad_s[3], out.rotor_speed_rad_s[1]);
  // Nose up: front motors M1, M3 faster.
  out = MixTorque(Vector3(0.0, 0.002, 0.0));
  EXPECT_GT(out.rotor_speed_rad_s[1], out.rotor_speed_rad_s[0]);
  EXPECT_GT(out.rotor_speed_rad_s[3], out.rotor_speed_rad_s[2]);
  // Positive (nose-right, clockwise from above) yaw torque comes from the
  // reaction of the CCW rotors M1, M2: they speed up.
  out = MixTorque(Vector3(0.0, 0.0, 0.001));
  EXPECT_GT(out.rotor_speed_rad_s[1], out.rotor_speed_rad_s[0]);
  EXPECT_GT(out.rotor_speed_rad_s[2], out.rotor_speed_rad_s[3]);
}

TEST_F(QuadXMixerTest, UnsaturatedCommandIsReproduced) {
  test::RandomStates random(8);
  for (int i = 0; i < 200; ++i) {
    const Vector3 torque(random.Uniform(-0.004, 0.004),
                         random.Uniform(-0.004, 0.004),
                         random.Uniform(-0.001, 0.001));
    const MixerOutput out = MixTorque(torque);
    ASSERT_FALSE(out.saturated);
    EXPECT_NEAR(out.achieved.thrust_n, hover_n_, 1e-12);
    EXPECT_LT((out.achieved.torque_frd_n_m - torque).norm(), 1e-12);
  }
}

TEST_F(QuadXMixerTest, MaximumThrustAndHoverMargin) {
  BodyWrenchCommand command;
  command.thrust_n = 10.0;
  const MixerOutput out = mixer_.Mix(command);
  EXPECT_TRUE(out.saturated);
  EXPECT_NEAR(out.achieved.thrust_n, config_.actuation_limits.thrust_max_n,
              1e-6);
  // Estimated thrust-to-weight ratio (docs/pluto_x_parameters.md): 1.04 N
  // against 68 g (drone + camera module) = 1.56; 1.77 without the camera.
  EXPECT_NEAR(out.achieved.thrust_n / hover_n_, 1.56, 0.01);
}

}  // namespace
}  // namespace pluto_x
