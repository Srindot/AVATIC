// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "pluto_x/control/legacy_pid_term.hpp"

namespace pluto_x {
namespace {

constexpr double kDt = 0.01;

LegacyPidGains Gains(double kp, double ki, double kd, double window) {
  LegacyPidGains gains;
  gains.kp = kp;
  gains.ki = ki;
  gains.kd = kd;
  gains.integral_window = window;
  return gains;
}

TEST(LegacyPidTerm, SumsProportionalIntegralAndMeasuredDerivative) {
  LegacyPidTerm term(Gains(2.0, 1.0, 0.5, 1.0), -0.1, 0.1);
  // I = 0.5 * 0.01 = 0.005; cp = 1.0; ci = 0.005; cd = 0.5 * 0.2 = 0.1
  EXPECT_NEAR(term.Update(0.5, 0.2, kDt), 1.105, 1e-12);
  EXPECT_NEAR(term.integral_accumulator(), 0.005, 1e-15);
}

TEST(LegacyPidTerm, IntegratesOnlyInsideWindow) {
  LegacyPidTerm term(Gains(2.0, 1.0, 0.0, 1.0), -0.1, 0.1);
  term.Update(0.5, 0.0, kDt);
  // |error| = 2 >= window: accumulator unchanged, cp = 4, ci = 0.005
  EXPECT_NEAR(term.Update(2.0, 0.0, kDt), 4.005, 1e-12);
  EXPECT_NEAR(term.integral_accumulator(), 0.005, 1e-15);
  // Window boundary is exclusive (legacy: abs(e) < lim).
  term.Update(1.0, 0.0, kDt);
  EXPECT_NEAR(term.integral_accumulator(), 0.005, 1e-15);
}

TEST(LegacyPidTerm, ClampsIntegralTermButNotAccumulator) {
  LegacyPidTerm term(Gains(0.0, 100.0, 0.0, 10.0), -0.1, 0.1);
  for (int i = 0; i < 10; ++i) {
    // ki * I = 100 * 0.01 * (i + 1) >= 1.0 > 0.1 from the first step on.
    EXPECT_NEAR(term.Update(1.0, 0.0, kDt), 0.1, 1e-12);
  }
  // Legacy behaviour: the accumulator keeps growing (0.1 after 10 steps).
  EXPECT_NEAR(term.integral_accumulator(), 0.1, 1e-12);
}

TEST(LegacyPidTerm, ResetClearsIntegral) {
  LegacyPidTerm term(Gains(0.0, 1.0, 0.0, 10.0), -1.0, 1.0);
  term.Update(1.0, 0.0, kDt);
  term.Reset();
  EXPECT_DOUBLE_EQ(term.integral_accumulator(), 0.0);
}

TEST(LegacyPidTerm, RejectsInvalidInput) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  LegacyPidTerm term(Gains(1.0, 1.0, 1.0, 1.0), -1.0, 1.0);
  EXPECT_THROW(term.Update(nan, 0.0, kDt), std::invalid_argument);
  EXPECT_THROW(term.Update(0.0, nan, kDt), std::invalid_argument);
  EXPECT_THROW(term.Update(0.0, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(term.Update(0.0, 0.0, -kDt), std::invalid_argument);
  EXPECT_THROW(LegacyPidTerm(Gains(nan, 0.0, 0.0, 0.0), -1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(LegacyPidTerm(Gains(1.0, 0.0, 0.0, 0.0), 1.0, -1.0),
               std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
