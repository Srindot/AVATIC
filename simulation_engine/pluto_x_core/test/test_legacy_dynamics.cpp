// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

class LegacyDynamicsTest : public ::testing::Test {
 protected:
  LegacyStackConfig config_ = test::LoadDefaultConfig();
  const LegacyVehicleParams& vehicle_ = config_.vehicle;
};

TEST_F(LegacyDynamicsTest, LevelHoverHasZeroAcceleration) {
  VehicleStateNed state;
  BodyWrenchCommand wrench;
  wrench.thrust_n = vehicle_.mass_kg * vehicle_.gravity_m_s2;
  const LegacyStateDerivatives d =
      ComputeLegacyDerivatives(state, wrench, 0.0, vehicle_);
  EXPECT_NEAR(d.acceleration_ned_m_s2.norm(), 0.0, 1e-12);
  EXPECT_NEAR(d.body_angular_accel_frd_rad_s2.norm(), 0.0, 1e-12);
}

TEST_F(LegacyDynamicsTest, RollTorqueUsesArmLengthTwice) {
  // Inherited quirk (legacy_double_arm_torque: true): p_dot = l * u2 / Jx.
  ASSERT_TRUE(vehicle_.legacy_double_arm_torque);
  VehicleStateNed state;
  BodyWrenchCommand wrench;
  wrench.torque_frd_n_m = Vector3(1.0, 0.0, 0.0);
  const LegacyStateDerivatives d =
      ComputeLegacyDerivatives(state, wrench, 0.0, vehicle_);
  EXPECT_NEAR(d.body_angular_accel_frd_rad_s2.x(),
              vehicle_.arm_length_m / vehicle_.inertia_diag_kg_m2.x(), 1e-12);
}

TEST_F(LegacyDynamicsTest, PhysicalTorqueWithoutLegacyQuirk) {
  const LegacyVehicleParams pluto = test::LoadPlutoXConfig().vehicle;
  ASSERT_FALSE(pluto.legacy_double_arm_torque);
  VehicleStateNed state;
  BodyWrenchCommand wrench;
  wrench.torque_frd_n_m = Vector3(0.001, -0.002, 0.0005);
  const LegacyStateDerivatives d =
      ComputeLegacyDerivatives(state, wrench, 0.0, pluto);
  EXPECT_TRUE(d.body_angular_accel_frd_rad_s2.isApprox(
      wrench.torque_frd_n_m.cwiseQuotient(pluto.inertia_diag_kg_m2), 1e-12));
}

TEST_F(LegacyDynamicsTest, PhysicalGyroscopicTorqueIsMinusOmegaCrossH) {
  LegacyVehicleParams v = test::LoadPlutoXConfig().vehicle;
  ASSERT_FALSE(v.legacy_gyroscopic_form);
  v.rotor_inertia_kg_m2 = 1e-4;
  const Vector3 w(0.3, -0.7, 1.1);
  const double o = 250.0;
  const Vector3 h(0.0, 0.0, -v.rotor_inertia_kg_m2 * o);
  EXPECT_TRUE(GyroscopicTorque(w, o, v).isApprox(-w.cross(h), 1e-12));
  // Legacy form reproduces kwad.cpp.
  v.legacy_gyroscopic_form = true;
  EXPECT_TRUE(GyroscopicTorque(w, o, v).isApprox(
      Vector3(-1e-4 * 0.3 * o, 1e-4 * -0.7 * o, 0.0), 1e-12));
}

TEST_F(LegacyDynamicsTest, DragActsOnAirRelativeVelocity) {
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(2.0, 0.0, 0.0);
  const Vector3 wind(2.0, 0.0, 0.0);  // moving with the air: no drag
  const ExternalWrenchNedFrd with_wind =
      ComputeLegacyExternalWrench(state, {}, 0.0, vehicle_, wind);
  EXPECT_NEAR(with_wind.force_ned_n.norm(), 0.0, 1e-12);
  const ExternalWrenchNedFrd calm =
      ComputeLegacyExternalWrench(state, {}, 0.0, vehicle_);
  EXPECT_NEAR(calm.force_ned_n.x(), -vehicle_.linear_drag_ned_n_s_m.x() * 2.0,
              1e-12);
}

// Core claim of the Gazebo port: the rigid-body Newton-Euler equations,
// driven by ComputeLegacyExternalWrench() plus gravity, reproduce the legacy
// equations of motion exactly.
void ExpectWrenchReproducesLegacyEquations(const LegacyVehicleParams& vehicle,
                                           double thrust_scale_n,
                                           double torque_scale_n_m) {
  test::RandomStates random(42);
  const Matrix3 inertia = vehicle.inertia_diag_kg_m2.asDiagonal();
  for (int i = 0; i < 500; ++i) {
    const VehicleStateNed state = random.State();
    BodyWrenchCommand wrench;
    wrench.thrust_n = random.Uniform(0.0, thrust_scale_n);
    wrench.torque_frd_n_m = random.UniformVector(torque_scale_n_m);
    const double o = random.Uniform(-200.0, 200.0);

    const LegacyStateDerivatives legacy =
        ComputeLegacyDerivatives(state, wrench, o, vehicle);
    const ExternalWrenchNedFrd external =
        ComputeLegacyExternalWrench(state, wrench, o, vehicle);

    const Vector3 accel_newton =
        external.force_ned_n / vehicle.mass_kg +
        Vector3(0.0, 0.0, vehicle.gravity_m_s2);
    const Vector3& w = state.body_rate_frd_rad_s;
    const Vector3 angular_accel_euler =
        inertia.inverse() *
        (external.torque_frd_n_m - w.cross(inertia * w));

    EXPECT_LT((accel_newton - legacy.acceleration_ned_m_s2).norm(), 1e-10);
    EXPECT_LT((angular_accel_euler - legacy.body_angular_accel_frd_rad_s2)
                  .norm(),
              1e-9 * std::max(1.0,
                              legacy.body_angular_accel_frd_rad_s2.norm()));
  }
}

TEST_F(LegacyDynamicsTest, ExternalWrenchReproducesLegacyEquations) {
  ExpectWrenchReproducesLegacyEquations(vehicle_, 40.0, 5.0);
}

TEST_F(LegacyDynamicsTest, ExternalWrenchReproducesEquationsPlutoX) {
  ExpectWrenchReproducesLegacyEquations(test::LoadPlutoXConfig().vehicle, 1.0,
                                        0.02);
}

TEST_F(LegacyDynamicsTest, EulerRatesEqualBodyRatesWhenLevel) {
  const EulerAnglesZyx rates =
      EulerRatesFromBodyRates({0.0, 0.0, 0.3}, Vector3(0.1, 0.2, 0.3));
  EXPECT_NEAR(rates.roll_rad, 0.1, 1e-12);
  EXPECT_NEAR(rates.pitch_rad, 0.2, 1e-12);
  EXPECT_NEAR(rates.yaw_rad, 0.3, 1e-12);
}

TEST_F(LegacyDynamicsTest, IntegratorFreeFallAndGroundClamp) {
  const LegacyReferenceIntegrator integrator(vehicle_,
                                             config_.reference_state_clamps);
  const double dt = 0.01;
  const double g = vehicle_.gravity_m_s2;

  // From 1 m altitude with no thrust: semi-implicit Euler step.
  VehicleStateNed state;
  state.position_ned_m.z() = -1.0;
  integrator.Step(state, BodyWrenchCommand{}, 0.0, dt);
  EXPECT_NEAR(state.velocity_ned_m_s.z(), g * dt, 1e-12);
  EXPECT_NEAR(state.position_ned_m.z(), -1.0 + g * dt * dt, 1e-12);

  // On the ground with no thrust: stays on the ground, no velocity build-up.
  VehicleStateNed grounded;
  for (int i = 0; i < 100; ++i) {
    integrator.Step(grounded, BodyWrenchCommand{}, 0.0, dt);
  }
  EXPECT_DOUBLE_EQ(grounded.position_ned_m.z(), 0.0);
  EXPECT_DOUBLE_EQ(grounded.velocity_ned_m_s.z(), 0.0);
}

TEST_F(LegacyDynamicsTest, IntegratorAppliesLegacyStateClamps) {
  const LegacyReferenceIntegrator integrator(vehicle_,
                                             config_.reference_state_clamps);
  VehicleStateNed state;
  state.position_ned_m.z() = -10.0;
  BodyWrenchCommand wrench;
  wrench.thrust_n = vehicle_.mass_kg * vehicle_.gravity_m_s2;
  wrench.torque_frd_n_m = Vector3(6.25, 0.0, 0.0);
  for (int i = 0; i < 500; ++i) {
    integrator.Step(state, wrench, 0.0, 0.01);
  }
  EXPECT_DOUBLE_EQ(state.body_rate_frd_rad_s.x(),
                   config_.reference_state_clamps.body_rate_limit_rad_s.x());
  EXPECT_DOUBLE_EQ(state.attitude.roll_rad,
                   config_.reference_state_clamps.attitude_limit_rad.roll_rad);
}

TEST_F(LegacyDynamicsTest, RejectsNonFiniteInput) {
  VehicleStateNed state;
  state.velocity_ned_m_s.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ComputeLegacyDerivatives(state, {}, 0.0, vehicle_),
               std::invalid_argument);
  EXPECT_THROW(ComputeLegacyExternalWrench(state, {}, 0.0, vehicle_),
               std::invalid_argument);
  const LegacyReferenceIntegrator integrator(vehicle_,
                                             config_.reference_state_clamps);
  VehicleStateNed ok;
  EXPECT_THROW(integrator.Step(ok, {}, 0.0, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
