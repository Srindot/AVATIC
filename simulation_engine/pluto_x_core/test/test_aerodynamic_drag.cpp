// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/dynamics/aerodynamic_drag.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

class AerodynamicDragTest : public ::testing::Test {
 protected:
  LegacyVehicleParams pluto_ = test::LoadPlutoXConfig().vehicle;
  double hover_rotor_sum_ = 4.0 * std::sqrt(0.5886 / (4.0 * 1.5e-8));
};

TEST_F(AerodynamicDragTest, LegacyLinearMatchesKwadCpp) {
  const LegacyVehicleParams legacy = test::LoadDefaultConfig().vehicle;
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(1.0, -2.0, 0.5);
  const Vector3 f = ComputeDragForceNed(state, Vector3::Zero(), 0.0, legacy);
  EXPECT_NEAR(f.x(), -0.16481 * 1.0, 1e-12);
  EXPECT_NEAR(f.y(), -0.31892 * -2.0, 1e-12);
  EXPECT_NEAR(f.z(), -0.0000011 * 0.5, 1e-15);
}

TEST_F(AerodynamicDragTest, NoRotorDragWithMotorsOffSoFallIsNearlyFree) {
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(0.0, 0.0, 1.0);  // falling at 1 m/s
  const Vector3 f = ComputeDragForceNed(state, Vector3::Zero(), 0.0, pluto_);
  // Only body drag: 1/2 rho CdA v^2 = 0.5 * 1.225 * 0.0096 * 1 = 5.88e-3 N
  EXPECT_NEAR(f.z(), -0.5 * 1.225 * 0.0096, 1e-12);
  EXPECT_LT(std::abs(f.z()) / (pluto_.mass_kg * pluto_.gravity_m_s2), 0.011);
}

TEST_F(AerodynamicDragTest, RotorDragAtHoverMatchesCalibration) {
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(1.0, 0.0, 0.0);  // 1 m/s north, level
  const Vector3 f =
      ComputeDragForceNed(state, Vector3::Zero(), hover_rotor_sum_, pluto_);
  const double body = 0.5 * 1.225 * 0.0044;
  EXPECT_NEAR(f.x(), -(0.0000064 * hover_rotor_sum_ + body), 1e-9);
  EXPECT_NEAR(-f.x() - body, 0.08, 0.001);  // rotor part ~0.08 N per m/s
  EXPECT_NEAR(f.y(), 0.0, 1e-12);
}

TEST_F(AerodynamicDragTest, RotorDragIsInPlaneOfTheBody) {
  // Level vehicle climbing: rotor drag has no vertical component.
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(0.0, 0.0, -2.0);
  LegacyVehicleParams no_body = pluto_;
  no_body.body_drag_area_frd_m2.setZero();
  EXPECT_NEAR(ComputeDragForceNed(state, Vector3::Zero(), hover_rotor_sum_,
                                  no_body).norm(),
              0.0, 1e-12);
  // Rotated vehicle: drag opposes velocity projected on the rotor plane.
  test::RandomStates random(21);
  for (int i = 0; i < 100; ++i) {
    const VehicleStateNed s = random.State();
    const Vector3 f =
        ComputeDragForceNed(s, Vector3::Zero(), hover_rotor_sum_, no_body);
    const Vector3 z_body =
        frames::RotationFromEulerZyx(s.attitude) * Vector3::UnitZ();
    EXPECT_NEAR(f.dot(z_body), 0.0, 1e-12);        // in the rotor plane
    EXPECT_LE(f.dot(s.velocity_ned_m_s), 1e-12);   // dissipative
  }
}

TEST_F(AerodynamicDragTest, WindMakesDragActOnAirRelativeVelocity) {
  VehicleStateNed state;
  state.velocity_ned_m_s = Vector3(3.0, 1.0, 0.0);
  EXPECT_NEAR(ComputeDragForceNed(state, state.velocity_ned_m_s,
                                  hover_rotor_sum_, pluto_).norm(),
              0.0, 1e-12);
}

TEST_F(AerodynamicDragTest, RejectsInvalidInput) {
  VehicleStateNed state;
  EXPECT_THROW(ComputeDragForceNed(state, Vector3::Zero(), -1.0, pluto_),
               std::invalid_argument);
  state.velocity_ned_m_s.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ComputeDragForceNed(state, Vector3::Zero(), 0.0, pluto_),
               std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
