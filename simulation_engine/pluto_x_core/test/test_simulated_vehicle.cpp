// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/control/legacy_flight_controller.hpp"
#include "pluto_x/simulation/simulated_vehicle.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

constexpr std::int64_t kStepNs = 1000000;  // 1 ms

LegacyStackConfig HoldConfig(LegacyStackConfig c) {
  c.pilot.initial_mode = FlightMode::kPositionHold;
  return c;
}

SimulatedVehicle MakeLegacyVehicle(const LegacyStackConfig& c) {
  return SimulatedVehicle(c, std::make_unique<LegacyFlightController>(c));
}

VehicleTruth Truth(const VehicleStateNed& state) {
  VehicleTruth truth;
  truth.state = state;
  return truth;
}

/// Records what SimulatedVehicle hands to the controller.
class ProbeController : public FlightController {
 public:
  explicit ProbeController(std::vector<FlightControllerInput>* log)
      : log_(log) {}
  std::string Name() const override { return "probe"; }
  FlightControllerOutput Step(const FlightControllerInput& input) override {
    log_->push_back(input);
    FlightControllerOutput output;
    output.motor_duty.fill(0.5);
    output.control_step_ran = true;
    return output;
  }

 private:
  std::vector<FlightControllerInput>* log_;
};

// With every new effect disabled (legacy config), SimulatedVehicle must
// apply exactly the wrench the controller stack + legacy dynamics give.
TEST(SimulatedVehicle, LegacyConfigMatchesDirectControllerPath) {
  const LegacyStackConfig c = HoldConfig(test::LoadDefaultConfig());
  SimulatedVehicle vehicle = MakeLegacyVehicle(c);
  LegacyControllerStack stack(c);
  test::RandomStates random(3);
  const VehicleStateNed state = random.State();
  const SimulatedVehicleStep out = vehicle.Step(0, kStepNs, Truth(state), std::nullopt);
  ControllerInput input;
  input.state = state;
  input.pilot.mode = FlightMode::kPositionHold;
  input.pilot.thrust_n = c.pilot.hover_thrust_n;
  const ControllerOutput direct = stack.Step(input);
  const ExternalWrenchNedFrd expected = ComputeLegacyExternalWrench(
      state, direct.mixer.achieved, direct.mixer.net_rotor_speed_rad_s,
      c.vehicle);
  EXPECT_TRUE(out.controller.control_step_ran);
  // duty = w / w_max and back: equal to rounding.
  EXPECT_LT((out.wrench.force_ned_n - expected.force_ned_n).norm(), 1e-9);
  EXPECT_LT((out.wrench.torque_frd_n_m - expected.torque_frd_n_m).norm(),
            1e-9);
}

TEST(SimulatedVehicle, ControllerRunsOncePerPeriod) {
  const LegacyStackConfig c = HoldConfig(test::LoadPlutoXConfig());
  SimulatedVehicle vehicle = MakeLegacyVehicle(c);
  int runs = 0;
  for (int i = 0; i < 100; ++i) {  // 100 ms
    runs += vehicle.Step(i * kStepNs, kStepNs, Truth(VehicleStateNed{}),
                         std::nullopt)
                .controller.control_step_ran;
  }
  EXPECT_EQ(runs, 10);  // 10 ms period
}

TEST(SimulatedVehicle, CommandLatencyDelaysRcFrames) {
  const LegacyStackConfig c = test::LoadPlutoXConfig();  // 50 ms latency
  ASSERT_NEAR(c.latency.command_s, 0.05, 1e-12);
  std::vector<FlightControllerInput> log;
  SimulatedVehicle vehicle(c, std::make_unique<ProbeController>(&log));
  RcFrame first;
  first.sequence = 1;
  first.channels_us.fill(1500);
  RcFrame second = first;
  second.sequence = 2;
  second.channels_us[2] = 1600;
  for (int i = 0; i <= 80; ++i) {
    // frame 1 from t = 0, frame 2 from t = 20 ms, re-sent every step
    vehicle.Step(i * kStepNs, kStepNs, Truth(VehicleStateNed{}),
                 i < 20 ? first : second);
  }
  EXPECT_FALSE(log[49].rc.has_value());
  ASSERT_TRUE(log[50].rc.has_value());
  EXPECT_EQ(log[50].rc->sequence, 1u);
  EXPECT_EQ(log[69].rc->sequence, 1u);
  EXPECT_EQ(log[70].rc->sequence, 2u);
  EXPECT_EQ(log[80].rc->channels_us[2], 1600);
  EXPECT_EQ(log[80].time_ns, 80 * kStepNs);
  EXPECT_EQ(log[80].timestep_ns, kStepNs);
}

TEST(SimulatedVehicle, FeedsBatteryAndRequiresController) {
  const LegacyStackConfig c = test::LoadPlutoXConfig();
  ASSERT_TRUE(c.battery.enabled);
  std::vector<FlightControllerInput> log;
  SimulatedVehicle vehicle(c, std::make_unique<ProbeController>(&log));
  for (int i = 0; i < 3; ++i) {
    vehicle.Step(i * kStepNs, kStepNs, Truth(VehicleStateNed{}), std::nullopt);
  }
  EXPECT_GT(log[2].battery_voltage_v, 3.0);
  EXPECT_GT(log[2].battery_current_a, 0.0);  // 50 % duty draws current
  EXPECT_THROW(SimulatedVehicle(c, nullptr), std::invalid_argument);
}

TEST(SimulatedVehicle, DeterministicWithAllEffectsEnabled) {
  const LegacyStackConfig c = HoldConfig(test::LoadPlutoXConfig());
  SimulatedVehicle a = MakeLegacyVehicle(c);
  SimulatedVehicle b = MakeLegacyVehicle(c);
  test::RandomStates random(4);
  for (int i = 0; i < 500; ++i) {
    const VehicleStateNed state = random.State();
    const auto oa = a.Step(i * kStepNs, kStepNs, Truth(state), std::nullopt);
    const auto ob = b.Step(i * kStepNs, kStepNs, Truth(state), std::nullopt);
    ASSERT_EQ(oa.wrench.force_ned_n, ob.wrench.force_ned_n);
    ASSERT_EQ(oa.wrench.torque_frd_n_m, ob.wrench.torque_frd_n_m);
  }
}

TEST(SimulatedVehicle, InvalidStateGivesZeroWrenchAndSafeCommand) {
  const LegacyStackConfig c = HoldConfig(test::LoadPlutoXConfig());
  SimulatedVehicle vehicle = MakeLegacyVehicle(c);
  VehicleStateNed bad;
  bad.velocity_ned_m_s.x() = std::numeric_limits<double>::quiet_NaN();
  const auto out = vehicle.Step(0, kStepNs, Truth(bad), std::nullopt);
  EXPECT_FALSE(out.controller.healthy);
  EXPECT_TRUE(out.wrench.force_ned_n.isZero());
  for (double duty : out.controller.motor_duty) {
    EXPECT_DOUBLE_EQ(duty, 0.0);
  }
  EXPECT_THROW(
      vehicle.Step(-1, kStepNs, Truth(VehicleStateNed{}), std::nullopt),
      std::invalid_argument);
  EXPECT_THROW(vehicle.Step(10, 0, Truth(VehicleStateNed{}), std::nullopt),
               std::invalid_argument);
}

// Closed loop on the reference integrator with every effect enabled: the
// hold must stay stable and bounded. It is not expected to be exact:
//  * battery sag: the legacy altitude loop has no integral term (z_ki = 0),
//    so the fresh battery's extra thrust gives a ~0.2 m altitude offset;
//  * wind gusts: ~0.1 m of horizontal wandering.
// (Effect-by-effect breakdown: docs/pluto_x_parameters.md, section 7.)
TEST(SimulatedVehicle, PlutoXPositionHoldWithAllEffects) {
  const LegacyStackConfig c = HoldConfig(test::LoadPlutoXConfig());
  SimulatedVehicle vehicle = MakeLegacyVehicle(c);
  const LegacyReferenceIntegrator integrator(c.vehicle,
                                             c.reference_state_clamps);
  VehicleStateNed state;
  const Vector3 target(1.0, 1.0, 1.0);
  double late_max_error_m = 0.0;
  for (int i = 0; i < 60000; ++i) {  // 60 s
    const auto out = vehicle.Step(i * kStepNs, kStepNs, Truth(state), std::nullopt);
    integrator.Step(state, out.propulsion.rotors.achieved,
                    out.propulsion.rotors.net_rotor_speed_rad_s, 0.001,
                    frames::SwapNedEnu(out.wind_enu_m_s));
    ASSERT_TRUE(IsFinite(state));
    if (i >= 50000) {  // last 10 s
      late_max_error_m = std::max(
          late_max_error_m,
          (frames::SwapNedEnu(state.position_ned_m) - target).norm());
    }
  }
  EXPECT_LT(late_max_error_m, 0.5);
  EXPECT_LT(state.velocity_ned_m_s.norm(), 0.5);
}

// Regression: spin-up reaction torque with a finite motor lag must not
// destabilise yaw. Without the legacy state clamps (which hid it), heading
// and altitude must hold exactly with the disturbance-free config.
TEST(SimulatedVehicle, PlutoXHoldsHeadingAndAltitudeWithoutStateClamps) {
  LegacyStackConfig c = test::LoadPlutoXConfig();
  c.battery.enabled = false;
  c.state_error.enabled = false;
  c.wind.enabled = false;
  ASSERT_TRUE(c.rotor.spin_up_reaction_torque);
  ASSERT_GT(c.rotor.time_constant_s, 0.0);
  c.reference_state_clamps.body_rate_limit_rad_s = Vector3(1e3, 1e3, 1e3);
  c.reference_state_clamps.attitude_limit_rad = {3.1, 1.5, 1e3};
  c.pilot.initial_mode = FlightMode::kPositionHold;
  SimulatedVehicle vehicle = MakeLegacyVehicle(c);
  const LegacyReferenceIntegrator integrator(c.vehicle,
                                             c.reference_state_clamps);
  VehicleStateNed state;
  for (int i = 0; i < 20000; ++i) {  // 20 s
    const auto out = vehicle.Step(i * kStepNs, kStepNs, Truth(state), std::nullopt);
    integrator.Step(state, out.propulsion.rotors.achieved,
                    out.propulsion.rotors.net_rotor_speed_rad_s, 0.001,
                    Vector3::Zero(),
                    out.propulsion.rotors.rotor_speed_sum_rad_s);
    ASSERT_TRUE(IsFinite(state));
  }
  const Vector3 final_enu = frames::SwapNedEnu(state.position_ned_m);
  EXPECT_LT((final_enu - Vector3(1.0, 1.0, 1.0)).norm(), 0.01);
  EXPECT_NEAR(state.attitude.yaw_rad, 0.0, 0.01);
}

}  // namespace
}  // namespace pluto_x
