// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <stdexcept>

#include "pluto_x/common/delay_line.hpp"

namespace pluto_x {
namespace {

TEST(DelayLine, ZeroDelayPassesThrough) {
  DelayLine<int> line(0, -1);
  line.Push(10, 5);
  EXPECT_EQ(line.Sample(10), 5);
}

TEST(DelayLine, ReleasesValuesAfterDelay) {
  DelayLine<int> line(50, -1);
  line.Push(0, 1);
  line.Push(20, 2);
  EXPECT_EQ(line.Sample(0), -1);   // initial value until t = 50
  EXPECT_EQ(line.Sample(49), -1);
  EXPECT_EQ(line.Sample(50), 1);
  EXPECT_EQ(line.Sample(69), 1);
  EXPECT_EQ(line.Sample(70), 2);
  EXPECT_EQ(line.Sample(1000), 2);  // holds the last released value
}

TEST(DelayLine, RejectsInvalidUse) {
  EXPECT_THROW(DelayLine<int>(-1, 0), std::invalid_argument);
  DelayLine<int> line(10, 0);
  line.Push(5, 1);
  EXPECT_THROW(line.Push(4, 2), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
