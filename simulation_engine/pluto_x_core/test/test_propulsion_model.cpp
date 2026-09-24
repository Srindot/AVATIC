// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "pluto_x/dynamics/propulsion_model.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

/// Duty that commands the given rotor speeds at the reference voltage.
std::array<double, kRotorCount> ToDuty(
    const std::array<double, kRotorCount>& speed_rad_s,
    const PropulsionModel& propulsion) {
  std::array<double, kRotorCount> duty{};
  for (int i = 0; i < kRotorCount; ++i) {
    duty[i] = speed_rad_s[i] / propulsion.max_rotor_speed_rad_s();
  }
  return duty;
}

TEST(PropulsionModel, LegacyConfigIsTransparent) {
  const LegacyStackConfig c = test::LoadDefaultConfig();
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  const QuadMixer mixer(c.rotor, c.vehicle.arm_length_m);
  BodyWrenchCommand command;
  command.thrust_n = 14.0;
  command.torque_frd_n_m = Vector3(0.3, -0.2, 0.01);
  const MixerOutput mixed = mixer.Mix(command);
  const PropulsionOutput out =
      propulsion.Step(ToDuty(mixed.rotor_speed_rad_s, propulsion), 0.001);
  EXPECT_DOUBLE_EQ(out.rotors.achieved.thrust_n, mixed.achieved.thrust_n);
  // speed -> duty -> speed round trip: equal to rounding
  EXPECT_LT((out.rotors.achieved.torque_frd_n_m -
             mixed.achieved.torque_frd_n_m).norm(),
            1e-12);
  EXPECT_DOUBLE_EQ(out.rotors.net_rotor_speed_rad_s,
                   mixed.net_rotor_speed_rad_s);
}

TEST(PropulsionModel, MotorLagDelaysThrust) {
  LegacyStackConfig c = test::LoadPlutoXConfig();
  c.battery.enabled = false;
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  const double w = 3000.0;
  PropulsionOutput out;
  for (int i = 0; i < 30; ++i) {  // one time constant (30 ms)
    out = propulsion.Step(ToDuty({w, w, w, w}, propulsion), 0.001);
  }
  const double w_actual = w * (1.0 - std::exp(-1.0));
  EXPECT_NEAR(out.rotors.rotor_speed_rad_s[0], w_actual, 1e-6);
  EXPECT_NEAR(out.rotors.achieved.thrust_n,
              4.0 * c.rotor.thrust_coefficient_n_s2 * w_actual * w_actual,
              1e-9);
}

TEST(PropulsionModel, BatterySagReducesThrustOverFlight) {
  LegacyStackConfig c = test::LoadPlutoXConfig();
  c.rotor.time_constant_s = 0.0;
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  // Command the speed that gives hover thrust at the reference voltage.
  const double hover_w = std::sqrt(c.battery.hover_thrust_n /
                                   (4.0 * c.rotor.thrust_coefficient_n_s2));
  const std::array<double, kRotorCount> cmd =
      ToDuty({hover_w, hover_w, hover_w, hover_w}, propulsion);
  const double thrust_fresh = propulsion.Step(cmd, 0.001).rotors.achieved.thrust_n;
  PropulsionOutput out;
  for (int i = 0; i < 480000; ++i) {  // 8 min, 1 ms steps
    out = propulsion.Step(cmd, 0.001);
  }
  // Fresh battery (4.2 V open circuit) gives more than reference thrust; a
  // battery near the end gives clearly less.
  EXPECT_GT(thrust_fresh, c.battery.hover_thrust_n * 1.2);
  EXPECT_LT(out.rotors.achieved.thrust_n, c.battery.hover_thrust_n);
  EXPECT_LT(out.battery_state_of_charge, 0.5);
}

TEST(PropulsionModel, SpinUpReactionTorqueOnYaw) {
  LegacyStackConfig c = test::LoadPlutoXConfig();
  c.battery.enabled = false;
  ASSERT_TRUE(c.rotor.spin_up_reaction_torque);
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  // Accelerate only the CCW rotors (M1, M2; s = +1) from rest for 1 ms.
  const double w = 3000.0;
  const PropulsionOutput out =
      propulsion.Step(ToDuty({0.0, w, w, 0.0}, propulsion), 0.001);
  const double dw = w * (1.0 - std::exp(-0.001 / c.rotor.time_constant_s));
  const double drag_torque =
      2.0 * c.rotor.yaw_drag_coefficient_n_m_s2 * dw * dw;
  const double spin_up = 2.0 * c.vehicle.rotor_inertia_kg_m2 * dw / 0.001;
  EXPECT_NEAR(out.rotors.achieved.torque_frd_n_m.z(), drag_torque + spin_up,
              1e-12);
  EXPECT_GT(spin_up, drag_torque);  // dominates during a fast spin-up
}

// Angular momentum bookkeeping: the yaw impulse delivered to the body by the
// spin-up torque equals the change of rotor angular momentum,
// sum(s_i J_rotor dw_i) (drag torque made negligible here).
TEST(PropulsionModel, SpinUpTorqueConservesAngularMomentum) {
  LegacyStackConfig c = test::LoadPlutoXConfig();
  c.battery.enabled = false;
  c.rotor.yaw_drag_coefficient_n_m_s2 = 1e-30;
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  const std::array<double, kRotorCount> start = {3000.0, 3100.0, 2900.0,
                                                 3050.0};
  const std::array<double, kRotorCount> end = {3400.0, 2800.0, 3200.0, 2700.0};
  for (int i = 0; i < 1000; ++i) {
    propulsion.Step(ToDuty(start, propulsion), 0.001);  // settle (1 s >> 30 ms)
  }
  double impulse = 0.0;
  PropulsionOutput out;
  for (int i = 0; i < 1000; ++i) {
    out = propulsion.Step(ToDuty(end, propulsion), 0.001);
    impulse += out.rotors.achieved.torque_frd_n_m.z() * 0.001;
  }
  // Signs: M0 -1, M1 +1, M2 +1, M3 -1 (magis_quad_x).
  const double expected = c.vehicle.rotor_inertia_kg_m2 *
                          (-(end[0] - start[0]) + (end[1] - start[1]) +
                           (end[2] - start[2]) - (end[3] - start[3]));
  EXPECT_NEAR(impulse, expected, 1e-9);
}

TEST(PropulsionModel, RejectsInvalidDuty) {
  const LegacyStackConfig c = test::LoadPlutoXConfig();
  PropulsionModel propulsion(c.rotor, c.vehicle.arm_length_m,
                             c.vehicle.rotor_inertia_kg_m2, c.battery);
  EXPECT_THROW(propulsion.Step({1.1, 0.0, 0.0, 0.0}, 0.001),
               std::invalid_argument);
  EXPECT_THROW(propulsion.Step({-0.1, 0.0, 0.0, 0.0}, 0.001),
               std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
