// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>

#include "pluto_x/common/frames.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

constexpr double kTolerance = 1e-12;
constexpr double kHalfPi = M_PI / 2.0;

void ExpectVectorNear(const Vector3& actual, const Vector3& expected,
                      double tolerance = kTolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

/// kwad.cpp rotateGFtoBF(), copied verbatim (renamed variables only).
Vector3 LegacyRotateGfToBf(double x, double y, double z, double phi,
                           double theta, double psi) {
  using std::cos;
  using std::sin;
  return Vector3(
      cos(psi) * cos(theta) * x + sin(psi) * cos(theta) * y - sin(theta) * z,
      (cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)) * x +
          (sin(phi) * sin(psi) * sin(theta) + cos(phi) * cos(psi)) * y +
          (cos(theta) * sin(phi)) * z,
      (cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi)) * x +
          (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)) * y +
          (cos(phi) * cos(theta)) * z);
}

TEST(Frames, SwapsAreInvolutions) {
  const Vector3 v(1.0, 2.0, 3.0);
  ExpectVectorNear(frames::SwapNedEnu(v), Vector3(2.0, 1.0, -3.0));
  ExpectVectorNear(frames::SwapNedEnu(frames::SwapNedEnu(v)), v);
  ExpectVectorNear(frames::SwapFrdFlu(v), Vector3(1.0, -2.0, -3.0));
  ExpectVectorNear(frames::SwapFrdFlu(frames::SwapFrdFlu(v)), v);
}

TEST(Frames, YawRotatesBodyXTowardsWorldY) {
  const Matrix3 r = frames::RotationFromEulerZyx({0.0, 0.0, kHalfPi});
  ExpectVectorNear(r * Vector3::UnitX(), Vector3::UnitY());
}

TEST(Frames, EulerRoundTrip) {
  test::RandomStates random(7);
  for (int i = 0; i < 200; ++i) {
    const EulerAnglesZyx euler{random.Uniform(-3.0, 3.0),
                               random.Uniform(-1.5, 1.5),
                               random.Uniform(-3.0, 3.0)};
    const EulerAnglesZyx recovered =
        frames::EulerZyxFromRotation(frames::RotationFromEulerZyx(euler));
    EXPECT_NEAR(recovered.roll_rad, euler.roll_rad, 1e-9);
    EXPECT_NEAR(recovered.pitch_rad, euler.pitch_rad, 1e-9);
    EXPECT_NEAR(recovered.yaw_rad, euler.yaw_rad, 1e-9);
  }
}

TEST(Frames, WorldToBodyMatchesLegacyRotateGfToBf) {
  test::RandomStates random(11);
  for (int i = 0; i < 200; ++i) {
    const Vector3 v = random.UniformVector(10.0);
    const EulerAnglesZyx e{random.Uniform(-1.0, 1.0),
                           random.Uniform(-1.0, 1.0),
                           random.Uniform(-3.0, 3.0)};
    ExpectVectorNear(frames::WorldToBodyZyx(v, e),
                     LegacyRotateGfToBf(v.x(), v.y(), v.z(), e.roll_rad,
                                        e.pitch_rad, e.yaw_rad),
                     1e-9);
  }
}

TEST(Frames, EnuIdentityAttitudeFacesEastInNed) {
  // Body x along ENU x (east) => NED yaw of +90 deg, level.
  const EulerAnglesZyx euler = frames::EulerZyxFromRotation(
      frames::RotationNedFrdFromEnuFlu(Matrix3::Identity()));
  EXPECT_NEAR(euler.roll_rad, 0.0, kTolerance);
  EXPECT_NEAR(euler.pitch_rad, 0.0, kTolerance);
  EXPECT_NEAR(euler.yaw_rad, kHalfPi, kTolerance);
}

TEST(Frames, EnuFacingNorthIsZeroNedYaw) {
  const Matrix3 r_enu = frames::RotationFromEulerZyx({0.0, 0.0, kHalfPi});
  const EulerAnglesZyx euler = frames::EulerZyxFromRotation(
      frames::RotationNedFrdFromEnuFlu(r_enu));
  EXPECT_NEAR(euler.yaw_rad, 0.0, kTolerance);
}

TEST(Frames, NedFrdRotationDescribesSamePhysicalAttitude) {
  test::RandomStates random(13);
  for (int i = 0; i < 100; ++i) {
    const Matrix3 r_enu_flu = frames::RotationFromEulerZyx(
        {random.Uniform(-3.0, 3.0), random.Uniform(-1.5, 1.5),
         random.Uniform(-3.0, 3.0)});
    const Matrix3 r_ned_frd = frames::RotationNedFrdFromEnuFlu(r_enu_flu);
    const Vector3 v_frd = random.UniformVector(1.0);
    // Map the FRD vector to the world in both conventions and compare.
    const Vector3 via_ned = r_ned_frd * v_frd;
    const Vector3 via_enu = frames::SwapNedEnu(
        r_enu_flu * frames::SwapFrdFlu(v_frd));
    ExpectVectorNear(via_ned, via_enu, 1e-12);
    // Result is a proper rotation.
    EXPECT_NEAR(r_ned_frd.determinant(), 1.0, 1e-12);
    // Inverse mapping recovers the original.
    EXPECT_TRUE(frames::RotationEnuFluFromNedFrd(r_ned_frd)
                    .isApprox(r_enu_flu, 1e-12));
  }
}

}  // namespace
}  // namespace pluto_x
