// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/dynamics/enu_adapter.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

Eigen::Quaterniond QuaternionFromEnuEuler(double roll, double pitch,
                                          double yaw) {
  return Eigen::Quaterniond(
      frames::RotationFromEulerZyx({roll, pitch, yaw}));
}

TEST(EnuAdapter, PositionAndVelocitySwapToNed) {
  RigidBodyStateEnu sim;
  sim.position_enu_m = Vector3(1.0, 2.0, 3.0);  // 1 E, 2 N, 3 up
  sim.velocity_enu_m_s = Vector3(0.1, 0.2, 0.3);
  sim.orientation_enu_flu = QuaternionFromEnuEuler(0.0, 0.0, M_PI / 2.0);
  const VehicleStateNed legacy = ToVehicleStateNed(sim);
  EXPECT_TRUE(legacy.position_ned_m.isApprox(Vector3(2.0, 1.0, -3.0)));
  EXPECT_TRUE(legacy.velocity_ned_m_s.isApprox(Vector3(0.2, 0.1, -0.3)));
  // Facing north in ENU => NED yaw 0.
  EXPECT_NEAR(legacy.attitude.yaw_rad, 0.0, 1e-12);
}

TEST(EnuAdapter, RollRightIsPositiveInBothConventions) {
  // FLU and FRD share the forward x axis, so a positive rotation about it
  // (right side down) is a positive roll in both conventions.
  RigidBodyStateEnu sim;
  sim.orientation_enu_flu = QuaternionFromEnuEuler(0.2, 0.0, M_PI / 2.0);
  const VehicleStateNed legacy = ToVehicleStateNed(sim);
  EXPECT_NEAR(legacy.attitude.roll_rad, 0.2, 1e-12);
  EXPECT_NEAR(legacy.attitude.pitch_rad, 0.0, 1e-12);
}

TEST(EnuAdapter, NoseUpIsPositiveFrdPitchButNegativeFluPitch) {
  // Nose up: FLU pitch is negative (right-hand rule about +y_left),
  // FRD pitch is positive (about +y_right).
  RigidBodyStateEnu sim;
  sim.orientation_enu_flu = QuaternionFromEnuEuler(0.0, -0.3, M_PI / 2.0);
  const VehicleStateNed legacy = ToVehicleStateNed(sim);
  EXPECT_NEAR(legacy.attitude.pitch_rad, 0.3, 1e-12);
}

TEST(EnuAdapter, BodyRatesConvertThroughBodyFrame) {
  RigidBodyStateEnu sim;
  sim.orientation_enu_flu = QuaternionFromEnuEuler(0.0, 0.0, M_PI / 2.0);
  // Facing north; world angular velocity about +North (= ENU y) is a roll
  // about the body forward axis.
  sim.angular_velocity_enu_rad_s = Vector3(0.0, 0.5, 0.0);
  const VehicleStateNed legacy = ToVehicleStateNed(sim);
  EXPECT_TRUE(legacy.body_rate_frd_rad_s.isApprox(Vector3(0.5, 0.0, 0.0),
                                                  1e-12));
}

// Full-chain check of the Gazebo formulation: apply the ENU-world wrench to
// an ENU rigid body (gravity along -z_ENU, inertia diag in FLU) and verify
// that the resulting accelerations, converted back to NED/FRD, equal the
// legacy equations of motion.
TEST(EnuAdapter, EnuRigidBodyReproducesLegacyEquations) {
  const LegacyStackConfig config = test::LoadDefaultConfig();
  const LegacyVehicleParams& v = config.vehicle;
  const Matrix3 inertia_body = v.inertia_diag_kg_m2.asDiagonal();
  test::RandomStates random(99);

  for (int i = 0; i < 300; ++i) {
    RigidBodyStateEnu sim;
    sim.position_enu_m = random.UniformVector(5.0);
    sim.velocity_enu_m_s = random.UniformVector(3.0);
    sim.orientation_enu_flu = QuaternionFromEnuEuler(
        random.Uniform(-1.0, 1.0), random.Uniform(-1.0, 1.0),
        random.Uniform(-3.0, 3.0));
    sim.angular_velocity_enu_rad_s = random.UniformVector(2.0);

    BodyWrenchCommand wrench;
    wrench.thrust_n = random.Uniform(0.0, 40.0);
    wrench.torque_frd_n_m = random.UniformVector(5.0);
    const double o = random.Uniform(-200.0, 200.0);

    const VehicleStateNed legacy_state = ToVehicleStateNed(sim);
    const LegacyStateDerivatives legacy =
        ComputeLegacyDerivatives(legacy_state, wrench, o, v);
    const WrenchEnuWorld applied = ToWrenchEnuWorld(
        ComputeLegacyExternalWrench(legacy_state, wrench, o, v),
        sim.orientation_enu_flu);

    // Rigid body in ENU world / FLU body.
    const Matrix3 r = sim.orientation_enu_flu.toRotationMatrix();
    const Vector3 accel_enu = applied.force_enu_n / v.mass_kg +
                              Vector3(0.0, 0.0, -v.gravity_m_s2);
    const Vector3 w_body = r.transpose() * sim.angular_velocity_enu_rad_s;
    const Vector3 torque_body = r.transpose() * applied.torque_enu_n_m;
    const Vector3 w_dot_body_flu =
        inertia_body.inverse() *
        (torque_body - w_body.cross(inertia_body * w_body));

    EXPECT_LT((frames::SwapNedEnu(accel_enu) - legacy.acceleration_ned_m_s2)
                  .norm(), 1e-9);
    EXPECT_LT((frames::SwapFrdFlu(w_dot_body_flu) -
               legacy.body_angular_accel_frd_rad_s2).norm(), 1e-9);
  }
}

TEST(EnuAdapter, RejectsInvalidState) {
  RigidBodyStateEnu sim;
  sim.velocity_enu_m_s.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ToVehicleStateNed(sim), std::invalid_argument);
  RigidBodyStateEnu degenerate;
  degenerate.orientation_enu_flu = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  EXPECT_THROW(ToVehicleStateNed(degenerate), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
