// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "pluto_x/common/backward_difference.hpp"

namespace pluto_x {
namespace {

TEST(BackwardDifference3, FirstSampleIsZeroThenDifference) {
  BackwardDifference3 diff;
  EXPECT_TRUE(diff.Update(Vector3(1.0, 2.0, 3.0), 0.01).isZero());
  const Vector3 d = diff.Update(Vector3(1.1, 1.9, 3.0), 0.01);
  EXPECT_NEAR(d.x(), 10.0, 1e-9);
  EXPECT_NEAR(d.y(), -10.0, 1e-9);
  EXPECT_NEAR(d.z(), 0.0, 1e-12);
  diff.Reset();
  EXPECT_TRUE(diff.Update(Vector3(5.0, 5.0, 5.0), 0.01).isZero());
}

TEST(BackwardDifference3, RejectsInvalidInput) {
  BackwardDifference3 diff;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(diff.Update(Vector3(nan, 0.0, 0.0), 0.01),
               std::invalid_argument);
  EXPECT_THROW(diff.Update(Vector3::Zero(), 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
