// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "pluto_x/arena/arena_scoring.hpp"
#include "pluto_x/arena/balloon_shape.hpp"

namespace pluto_x {
namespace {

// Unit-diameter sphere profile (radius 0.5), sampled.
std::vector<std::array<double, 2>> SphereProfile(int n = 200) {
  std::vector<std::array<double, 2>> p;
  for (int i = 0; i <= n; ++i) {
    const double a = -M_PI / 2 + M_PI * i / n;
    p.push_back({0.5 * std::cos(a), 0.5 * std::sin(a)});
  }
  return p;
}

TEST(BalloonShape, SphereDistancesMatchAnalytic) {
  const BalloonShape shape(SphereProfile(), 0.30);  // radius 0.15 m
  EXPECT_NEAR(shape.max_radius_m(), 0.15, 1e-9);
  EXPECT_TRUE(shape.Contains(Vector3(0.0, 0.0, 0.0)));
  EXPECT_TRUE(shape.Contains(Vector3(0.1, 0.0, 0.05)));
  EXPECT_FALSE(shape.Contains(Vector3(0.2, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(shape.DistanceToSurface(Vector3(0.05, 0.0, 0.0)), 0.0);
  // outside, in several directions: distance = |p| - 0.15 (polyline error small)
  for (const Vector3& p : {Vector3(0.5, 0, 0), Vector3(0, -0.4, 0.1),
                           Vector3(0.1, 0.1, 0.4), Vector3(0, 0, -0.3)}) {
    EXPECT_NEAR(shape.DistanceToSurface(p), p.norm() - 0.15, 1e-4) << p.transpose();
  }
  EXPECT_TRUE(shape.TouchedBySphere(Vector3(0.21, 0.0, 0.0), 0.07));
  EXPECT_FALSE(shape.TouchedBySphere(Vector3(0.23, 0.0, 0.0), 0.07));
}

TEST(BalloonShape, CapsAndValidation) {
  // cylinder r = 0.5, z in [-1, 1] (unit diameter): caps are flat
  const BalloonShape cyl({{0.5, -1.0}, {0.5, 1.0}}, 1.0);
  EXPECT_NEAR(cyl.DistanceToSurface(Vector3(0.0, 0.0, 1.3)), 0.3, 1e-12);
  EXPECT_NEAR(cyl.DistanceToSurface(Vector3(0.0, 0.0, -1.2)), 0.2, 1e-12);
  EXPECT_NEAR(cyl.DistanceToSurface(Vector3(0.8, 0.0, 0.0)), 0.3, 1e-12);
  EXPECT_NEAR(cyl.height_m(), 2.0, 1e-12);
  EXPECT_THROW(BalloonShape({{0.5, 0.0}}, 1.0), std::invalid_argument);
  EXPECT_THROW(BalloonShape({{0.5, 1.0}, {0.5, 0.0}}, 1.0), std::invalid_argument);
  EXPECT_THROW(BalloonShape({{-0.1, 0.0}, {0.5, 1.0}}, 1.0), std::invalid_argument);
  EXPECT_THROW(BalloonShape(SphereProfile(), 0.0), std::invalid_argument);
}

ArenaScoring MakeArena(double limit_s = 15.0) {
  std::vector<BalloonSpec> balloons = {
      {"b0", "red", 75, Vector3(1.0, 0.0, 1.0)},
      {"b1", "yellow", 25, Vector3(2.0, 0.0, 1.0)},
      {"b2", "green", 100, Vector3(0.0, 3.0, 1.5)},
  };
  return ArenaScoring(balloons, BalloonShape(SphereProfile(), 0.30),
                      {{Vector3::Zero(), 0.07}}, limit_s);
}

TEST(ArenaScoring, PopsOnceAndAddsPoints) {
  ArenaScoring arena = MakeArena();
  EXPECT_EQ(arena.max_points(), 200);
  EXPECT_TRUE(arena.Update(0.5, Vector3(1.0, 0.0, 1.0)).empty());  // clock not started
  arena.StartClock(1.0);
  EXPECT_TRUE(arena.Update(1.1, Vector3(0.0, 0.0, 1.0)).empty());
  auto events = arena.Update(1.2, Vector3(0.79, 0.0, 1.0));  // 0.21 m: touches
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].index, 0u);
  EXPECT_EQ(events[0].points, 75);
  EXPECT_NEAR(events[0].run_time_s, 0.2, 1e-12);
  EXPECT_TRUE(arena.Update(1.3, Vector3(1.0, 0.0, 1.0)).empty());  // already popped
  events = arena.Update(1.4, Vector3(2.0, 0.0, 1.0));
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].total_points, 100);
  EXPECT_EQ(arena.total_points(), 100);
  EXPECT_EQ(arena.popped_count(), 2u);
  EXPECT_TRUE(arena.popped(1));
  EXPECT_FALSE(arena.popped(2));
}

TEST(ArenaScoring, TimeLimitEndsTheRunExactly) {
  ArenaScoring arena = MakeArena(15.0);
  arena.StartClock(2.0);
  arena.StartClock(5.0);  // ignored
  EXPECT_NEAR(arena.remaining_s(10.0), 7.0, 1e-12);
  EXPECT_FALSE(arena.finished());
  // at exactly 15 s of run time the run is over: this contact does not score
  EXPECT_TRUE(arena.Update(17.0, Vector3(1.0, 0.0, 1.0)).empty());
  EXPECT_TRUE(arena.finished());
  EXPECT_EQ(arena.total_points(), 0);
  EXPECT_DOUBLE_EQ(arena.remaining_s(18.0), 0.0);
  EXPECT_TRUE(arena.Update(17.1, Vector3(2.0, 0.0, 1.0)).empty());
}

TEST(ArenaScoring, RejectsBadParameters) {
  const BalloonShape shape(SphereProfile(), 0.3);
  const std::vector<ContactSphere> one = {{Vector3::Zero(), 0.07}};
  EXPECT_THROW(ArenaScoring({}, shape, {}, 15.0), std::invalid_argument);
  EXPECT_THROW(ArenaScoring({}, shape, {{Vector3::Zero(), 0.0}}, 15.0),
               std::invalid_argument);
  EXPECT_THROW(ArenaScoring({}, shape, one, 0.0), std::invalid_argument);

}

// Penalty balloons subtract; the best achievable score excludes them.
TEST(ArenaScoring, PenaltyBalloonsSubtract) {
  const std::vector<BalloonSpec> balloons = {
      {"b0", "green", 100, Vector3(1.0, 0.0, 1.0)},
      {"b1", "red", -75, Vector3(2.0, 0.0, 1.0)},
  };
  ArenaScoring arena(balloons, BalloonShape(SphereProfile(), 0.30),
                     {{Vector3::Zero(), 0.07}}, 15.0);
  EXPECT_EQ(arena.max_points(), 100);
  arena.StartClock(0.0);
  EXPECT_EQ(arena.Update(1.0, Vector3(1.0, 0.0, 1.0)).at(0).total_points, 100);
  const auto red = arena.Update(2.0, Vector3(2.0, 0.0, 1.0));
  ASSERT_EQ(red.size(), 1u);
  EXPECT_EQ(red[0].points, -75);
  EXPECT_EQ(arena.total_points(), 25);
}

// Off-centre contact spheres move and rotate with the vehicle.
TEST(ArenaScoring, ContactSpheresFollowAttitude) {
  const std::vector<BalloonSpec> balloons = {{"b0", "red", 75, Vector3(0.40, 0.0, 1.0)}};
  // a "guard" sphere 0.10 m ahead of the reference point
  const std::vector<ContactSphere> spheres = {{Vector3(0.10, 0.0, 0.0), 0.03}};
  ArenaScoring facing(balloons, BalloonShape(SphereProfile(), 0.30), spheres, 15.0);
  facing.StartClock(0.0);
  // reference 0.17 m from the balloon centre: the guard is at 0.07 m,
  // i.e. 0.08 m inside the surface -> pop
  EXPECT_EQ(facing.Update(0.1, Vector3(0.23, 0.0, 1.0)).size(), 1u);

  ArenaScoring turned(balloons, BalloonShape(SphereProfile(), 0.30), spheres, 15.0);
  turned.StartClock(0.0);
  Matrix3 yaw180 = Matrix3::Identity();
  yaw180(0, 0) = yaw180(1, 1) = -1.0;  // guard now points away from the balloon
  // reference 0.17 m away, guard 0.27 m away: its surface gap is 0.12 m > 0.03
  EXPECT_TRUE(turned.Update(0.1, Vector3(0.23, 0.0, 1.0), yaw180).empty());
}

}  // namespace
}  // namespace pluto_x
