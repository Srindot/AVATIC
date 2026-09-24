// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <limits>

#include "pluto_x/control/pilot_command.hpp"

namespace pluto_x {
namespace {

TEST(PilotCommand, IsFiniteChecksAllFields) {
  PilotCommand command;
  EXPECT_TRUE(IsFinite(command));
  command.pitch_setpoint_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsFinite(command));
}

}  // namespace
}  // namespace pluto_x
