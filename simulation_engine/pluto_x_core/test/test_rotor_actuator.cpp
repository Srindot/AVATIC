// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluto_x/dynamics/rotor_actuator.hpp"

namespace pluto_x {
namespace {

TEST(RotorActuator, ZeroTimeConstantIsInstantaneous) {
  RotorActuator rotor(0.0);
  EXPECT_DOUBLE_EQ(rotor.Step(1234.0, 0.001), 1234.0);
}

TEST(RotorActuator, FirstOrderStepResponse) {
  RotorActuator rotor(0.03);
  double speed = 0.0;
  for (int i = 0; i < 30; ++i) {  // 30 ms = one time constant
    speed = rotor.Step(1000.0, 0.001);
  }
  EXPECT_NEAR(speed, 1000.0 * (1.0 - std::exp(-1.0)), 1e-9);
}

TEST(RotorActuator, ExactDiscretisationIsStepSizeIndependent) {
  RotorActuator fine(0.03);
  RotorActuator coarse(0.03);
  for (int i = 0; i < 100; ++i) {
    fine.Step(500.0, 0.001);
  }
  coarse.Step(500.0, 0.1);
  EXPECT_NEAR(fine.speed_rad_s(), coarse.speed_rad_s(), 1e-9);
  // A step much longer than tau does not overshoot.
  RotorActuator rotor(0.03);
  EXPECT_LE(rotor.Step(500.0, 10.0), 500.0);
}

TEST(RotorActuator, RejectsInvalidInput) {
  EXPECT_THROW(RotorActuator(-0.1), std::invalid_argument);
  RotorActuator rotor(0.03);
  EXPECT_THROW(rotor.Step(-1.0, 0.001), std::invalid_argument);
  EXPECT_THROW(rotor.Step(std::numeric_limits<double>::quiet_NaN(), 0.001),
               std::invalid_argument);
  EXPECT_THROW(rotor.Step(1.0, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
