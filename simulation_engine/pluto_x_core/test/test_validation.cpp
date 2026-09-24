// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(Validation, IsFiniteRejectsNanAndInf) {
  EXPECT_TRUE(IsFinite(1.0));
  EXPECT_FALSE(IsFinite(kNan));
  EXPECT_FALSE(IsFinite(-kInf));
  EXPECT_TRUE(IsFinite(Vector3(1.0, 2.0, 3.0)));
  EXPECT_FALSE(IsFinite(Vector3(1.0, kNan, 3.0)));
  EXPECT_FALSE(IsFinite(EulerAnglesZyx{0.0, 0.0, kInf}));
}

TEST(Validation, ClampBehaviour) {
  EXPECT_DOUBLE_EQ(Clamp(5.0, 0.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(Clamp(-5.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(Clamp(0.5, 0.0, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(ClampSymmetric(-3.0, 2.0), -2.0);
}

TEST(Validation, ClampRejectsInvalidBounds) {
  EXPECT_THROW(Clamp(0.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(Clamp(0.0, kNan, 1.0), std::invalid_argument);
  EXPECT_THROW(ClampSymmetric(0.0, -1.0), std::invalid_argument);
  EXPECT_THROW(ClampSymmetric(0.0, kNan), std::invalid_argument);
}

TEST(Validation, RequireFiniteThrowsWithName) {
  try {
    RequireFinite(kNan, "mass");
    FAIL() << "expected exception";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("mass"), std::string::npos);
  }
}

}  // namespace
}  // namespace pluto_x
