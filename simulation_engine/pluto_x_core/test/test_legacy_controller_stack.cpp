// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <limits>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/control/legacy_controller_stack.hpp"
#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

class LegacyControllerStackTest : public ::testing::Test {
 protected:
  LegacyStackConfig config_ = test::LoadDefaultConfig();
  double weight_n_ = config_.vehicle.mass_kg * config_.vehicle.gravity_m_s2;
};

TEST_F(LegacyControllerStackTest, TargetConvertedFromEnuToNed) {
  LegacyControllerStack stack(config_);
  EXPECT_TRUE(stack.position_hold_target_ned_m().isApprox(
      Vector3(1.0, 1.0, -1.0)));
}

TEST_F(LegacyControllerStackTest, PositionHoldEquilibriumAtTarget) {
  LegacyControllerStack stack(config_);
  ControllerInput input;
  input.pilot.mode = FlightMode::kPositionHold;
  input.state.position_ned_m = stack.position_hold_target_ned_m();
  const ControllerOutput out = stack.Step(input);
  EXPECT_EQ(out.status, ControllerStatus::kOk);
  EXPECT_NEAR(out.mixer.achieved.thrust_n, weight_n_, 1e-9);
  EXPECT_NEAR(out.mixer.achieved.torque_frd_n_m.norm(), 0.0, 1e-9);
}

TEST_F(LegacyControllerStackTest, ManualModePassesPilotCommand) {
  LegacyControllerStack stack(config_);
  ControllerInput input;
  input.pilot.mode = FlightMode::kManualAttitude;
  input.pilot.thrust_n = 15.0;
  input.pilot.roll_setpoint_rad = 0.1;
  const ControllerOutput out = stack.Step(input);
  EXPECT_NEAR(out.requested.thrust_n, 15.0, 1e-12);
  EXPECT_NEAR(out.attitude_setpoint.roll_rad, 0.1, 1e-12);
  EXPECT_GT(out.body_rate_setpoint_frd_rad_s.x(), 0.0);
  EXPECT_GT(out.requested.torque_frd_n_m.x(), 0.0);
}

TEST_F(LegacyControllerStackTest, HoldsInitialHeading) {
  ASSERT_TRUE(config_.pilot.hold_initial_heading);
  LegacyControllerStack stack(config_);
  ControllerInput input;
  input.pilot.mode = FlightMode::kManualAttitude;
  input.pilot.thrust_n = weight_n_;
  input.state.attitude.yaw_rad = 1.2;
  EXPECT_DOUBLE_EQ(stack.Step(input).attitude_setpoint.yaw_rad, 1.2);
  input.state.attitude.yaw_rad = 1.0;  // later heading changes are errors
  const ControllerOutput out = stack.Step(input);
  EXPECT_DOUBLE_EQ(out.attitude_setpoint.yaw_rad, 1.2);
  EXPECT_GT(out.body_rate_setpoint_frd_rad_s.z(), 0.0);  // turns back

  LegacyStackConfig north = config_;
  north.pilot.hold_initial_heading = false;
  LegacyControllerStack legacy(north);
  EXPECT_DOUBLE_EQ(legacy.Step(input).attitude_setpoint.yaw_rad, 0.0);
}

TEST_F(LegacyControllerStackTest, NonFiniteInputCommandsSafeOutput) {
  LegacyControllerStack stack(config_);
  ControllerInput input;
  input.state.attitude.roll_rad = std::numeric_limits<double>::quiet_NaN();
  input.pilot.thrust_n = 20.0;
  const ControllerOutput out = stack.Step(input);
  EXPECT_EQ(out.status, ControllerStatus::kInvalidInput);
  EXPECT_DOUBLE_EQ(out.mixer.achieved.thrust_n,
                   config_.actuation_limits.thrust_min_n);
  for (double w : out.mixer.rotor_speed_rad_s) {
    EXPECT_DOUBLE_EQ(w, 0.0);
  }
}

/// Runs position hold from rest on the ground for duration_s on the
/// reference integrator and returns the final state.
VehicleStateNed RunPositionHold(const LegacyStackConfig& config,
                                double duration_s) {
  LegacyControllerStack stack(config);
  const LegacyReferenceIntegrator integrator(config.vehicle,
                                             config.reference_state_clamps);
  VehicleStateNed state;
  ControllerInput input;
  input.pilot.mode = FlightMode::kPositionHold;
  const double dt = stack.period_s();
  const int steps = static_cast<int>(duration_s / dt);
  for (int i = 0; i < steps; ++i) {
    input.state = state;
    const ControllerOutput out = stack.Step(input);
    EXPECT_EQ(out.status, ControllerStatus::kOk);
    const LegacyStateDerivatives d = integrator.Step(
        state, out.mixer.achieved, out.mixer.net_rotor_speed_rad_s, dt);
    input.body_angular_accel_frd_rad_s2 = d.body_angular_accel_frd_rad_s2;
  }
  return state;
}

// Closed loop on the legacy reference integrator: from rest on the ground,
// position hold (yaw-only error frame) reaches the (1, 1, 1) m ENU target.
TEST_F(LegacyControllerStackTest, ClosedLoopPositionHoldReachesTarget) {
  ASSERT_EQ(config_.controller.position.error_frame,
            PositionErrorFrame::kYawOnly);
  const VehicleStateNed state = RunPositionHold(config_, 30.0);
  ASSERT_TRUE(IsFinite(state));
  const Vector3 final_enu = frames::SwapNedEnu(state.position_ned_m);
  EXPECT_LT((final_enu - Vector3(1.0, 1.0, 1.0)).norm(), 0.05)
      << "final ENU position " << final_enu.transpose();
  EXPECT_LT(state.velocity_ned_m_s.norm(), 0.05);
}

// Same closed-loop scenario with the estimated Pluto X parameters.
TEST_F(LegacyControllerStackTest, PlutoXClosedLoopPositionHoldReachesTarget) {
  const LegacyStackConfig pluto = test::LoadPlutoXConfig();
  const VehicleStateNed state = RunPositionHold(pluto, 30.0);
  ASSERT_TRUE(IsFinite(state));
  const Vector3 final_enu = frames::SwapNedEnu(state.position_ned_m);
  EXPECT_LT((final_enu - Vector3(1.0, 1.0, 1.0)).norm(), 0.05)
      << "final ENU position " << final_enu.transpose();
  EXPECT_LT(state.velocity_ned_m_s.norm(), 0.05);
}

// Documents the defect found while porting: the kwad.cpp full-attitude error
// frame feeds sin(tilt) * altitude back into the horizontal error, with a
// destabilising sign above real ground. With the shipped inertia it holds at
// 1 m but diverges for a 2 m target; the yaw-only frame converges there. If
// this test starts failing, revisit docs/legacy_port.md.
TEST_F(LegacyControllerStackTest, LegacyFullAttitudeFrameDivergesAtTwoMetres) {
  LegacyStackConfig legacy = config_;
  legacy.pilot.position_hold_target_enu_m = Vector3(1.0, 1.0, 2.0);
  legacy.controller.position.error_frame =
      PositionErrorFrame::kLegacyFullAttitude;
  const VehicleStateNed diverged = RunPositionHold(legacy, 60.0);
  EXPECT_GT((frames::SwapNedEnu(diverged.position_ned_m) -
             Vector3(1.0, 1.0, 2.0)).norm(), 10.0);

  legacy.controller.position.error_frame = PositionErrorFrame::kYawOnly;
  const VehicleStateNed converged = RunPositionHold(legacy, 60.0);
  EXPECT_LT((frames::SwapNedEnu(converged.position_ned_m) -
             Vector3(1.0, 1.0, 2.0)).norm(), 0.01);
}

}  // namespace
}  // namespace pluto_x
