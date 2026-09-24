// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>

#include "pluto_x/control/legacy_cascade.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

class LegacyCascadeTest : public ::testing::Test {
 protected:
  LegacyStackConfig config_ = test::LoadDefaultConfig();
  double dt_ = config_.controller.period_s;
  double weight_n_ = config_.vehicle.mass_kg * config_.vehicle.gravity_m_s2;

  LegacyPositionController MakePosition() const {
    return LegacyPositionController(config_.controller.position,
                                    config_.actuation_limits, config_.vehicle);
  }
};

TEST_F(LegacyCascadeTest, PositionAtTargetCommandsWeight) {
  auto controller = MakePosition();
  VehicleStateNed state;
  state.position_ned_m = Vector3(1.0, 1.0, -1.0);
  const PositionControllerOutput out =
      controller.Update(state, state.position_ned_m, dt_);
  EXPECT_NEAR(out.roll_setpoint_rad, 0.0, 1e-12);
  EXPECT_NEAR(out.pitch_setpoint_rad, 0.0, 1e-12);
  EXPECT_NEAR(out.thrust_n, weight_n_, 1e-12);
}

TEST_F(LegacyCascadeTest, PositionErrorProducesCorrectTiltDirection) {
  auto controller = MakePosition();
  VehicleStateNed state;
  // Target 0.1 m north: nose down (negative FRD pitch).
  PositionControllerOutput out =
      controller.Update(state, Vector3(0.1, 0.0, 0.0), dt_);
  EXPECT_LT(out.pitch_setpoint_rad, 0.0);
  EXPECT_NEAR(out.roll_setpoint_rad, 0.0, 1e-12);

  controller.Reset();
  // Target 0.1 m east: right side down (positive FRD roll).
  out = controller.Update(state, Vector3(0.0, 0.1, 0.0), dt_);
  EXPECT_GT(out.roll_setpoint_rad, 0.0);

  controller.Reset();
  // Target 0.1 m above (NED z negative): more than hover thrust.
  out = controller.Update(state, Vector3(0.0, 0.0, -0.1), dt_);
  // cp = 5.88 * 0.1 = 0.588 => thrust = weight + 0.588
  EXPECT_NEAR(out.thrust_n, weight_n_ + 0.588, 1e-9);
}

TEST_F(LegacyCascadeTest, PositionOutputsSaturate) {
  auto controller = MakePosition();
  VehicleStateNed state;
  const PositionControllerOutput out =
      controller.Update(state, Vector3(100.0, -100.0, -100.0), dt_);
  EXPECT_DOUBLE_EQ(out.pitch_setpoint_rad,
                   -config_.controller.position.pitch_limit_rad);
  EXPECT_DOUBLE_EQ(out.roll_setpoint_rad,
                   -config_.controller.position.roll_limit_rad);
  EXPECT_DOUBLE_EQ(out.thrust_n, config_.actuation_limits.thrust_max_n);
}

TEST_F(LegacyCascadeTest, InvertedVehicleCommandsMinimumThrust) {
  auto controller = MakePosition();
  VehicleStateNed state;
  state.attitude.roll_rad = M_PI;  // upside down: cos(roll) cos(pitch) < 0
  const PositionControllerOutput out =
      controller.Update(state, Vector3(0.0, 0.0, -1.0), dt_);
  EXPECT_DOUBLE_EQ(out.thrust_n, config_.actuation_limits.thrust_min_n);
}

TEST_F(LegacyCascadeTest, TargetIsNotMutatedAcrossStepsWithYaw) {
  // kwad.cpp overwrote x_des/y_des with their yaw-rotated copies each step.
  // With ki = 0 and a constant state, consecutive outputs must be identical.
  LegacyPositionControllerParams params = config_.controller.position;
  params.north.ki = params.east.ki = params.down.ki = 0.0;
  LegacyPositionController controller(params, config_.actuation_limits,
                                      config_.vehicle);
  VehicleStateNed state;
  state.attitude.yaw_rad = 1.0;
  const Vector3 target(0.2, 0.1, -0.1);
  const PositionControllerOutput first = controller.Update(state, target, dt_);
  const PositionControllerOutput second = controller.Update(state, target, dt_);
  EXPECT_DOUBLE_EQ(first.roll_setpoint_rad, second.roll_setpoint_rad);
  EXPECT_DOUBLE_EQ(first.pitch_setpoint_rad, second.pitch_setpoint_rad);
  EXPECT_DOUBLE_EQ(first.thrust_n, second.thrust_n);
}

TEST_F(LegacyCascadeTest, AttitudeLoopProportionalAndClamped) {
  LegacyAttitudeController controller(config_.controller.attitude);
  VehicleStateNed state;
  // 0.1 rad roll error: p_sp = 4.5 * 0.1 = 0.45 (< 0.87266 limit)
  Vector3 rate_sp = controller.Update(state, {0.1, 0.0, 0.0}, dt_);
  EXPECT_NEAR(rate_sp.x(), 0.45, 1e-12);
  EXPECT_NEAR(rate_sp.y(), 0.0, 1e-12);
  // 1 rad pitch error saturates at q_max.
  rate_sp = controller.Update(state, {0.0, -1.0, 0.0}, dt_);
  EXPECT_DOUBLE_EQ(rate_sp.y(),
                   -config_.controller.attitude.body_rate_limit_rad_s.y());
}

TEST_F(LegacyCascadeTest, RateLoopUsesMeasuredAngularAcceleration) {
  LegacyRateController controller(config_.controller.rate,
                                  config_.actuation_limits);
  // error p = 0.1: cp = 0.27, ci = 1.0 * 0.1*0.01 = 0.001,
  // cd = -0.01 * 2.0 = -0.02  => 0.251
  const Vector3 torque = controller.Update(
      Vector3::Zero(), Vector3(2.0, 0.0, 0.0), Vector3(0.1, 0.0, 0.0), dt_);
  EXPECT_NEAR(torque.x(), 0.251, 1e-12);
  EXPECT_NEAR(torque.y(), 0.0, 1e-12);
}

TEST_F(LegacyCascadeTest, RateLoopSaturatesAtTorqueLimits) {
  LegacyRateController controller(config_.controller.rate,
                                  config_.actuation_limits);
  const Vector3 torque = controller.Update(
      Vector3::Zero(), Vector3::Zero(), Vector3(100.0, -100.0, 100.0), dt_);
  const Vector3& limit = config_.actuation_limits.torque_limit_n_m;
  EXPECT_DOUBLE_EQ(torque.x(), limit.x());
  EXPECT_DOUBLE_EQ(torque.y(), -limit.y());
  EXPECT_DOUBLE_EQ(torque.z(), limit.z());
}

}  // namespace
}  // namespace pluto_x
