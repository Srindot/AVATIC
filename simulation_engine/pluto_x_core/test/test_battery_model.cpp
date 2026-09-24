// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "pluto_x/dynamics/battery_model.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

BatteryParams PlutoBattery() { return test::LoadPlutoXConfig().battery; }

TEST(BatteryModel, DisabledHasUnitThrustScale) {
  const BatteryModel battery(test::LoadDefaultConfig().battery);
  EXPECT_FALSE(battery.enabled());
  EXPECT_DOUBLE_EQ(battery.thrust_scale(), 1.0);
}

TEST(BatteryModel, FullBatteryAtRestIsTableTopVoltage) {
  const BatteryModel battery(PlutoBattery());
  EXPECT_DOUBLE_EQ(battery.voltage_v(), 4.20);
  EXPECT_NEAR(battery.thrust_scale(), std::pow(4.2 / 3.7, 2), 1e-12);
}

TEST(BatteryModel, HoverDischargeFollowsCapacity) {
  const BatteryParams params = PlutoBattery();
  BatteryModel battery(params);
  for (int i = 0; i < 60000; ++i) {  // 60 s at hover, 1 ms steps
    battery.Step(params.hover_thrust_n, 0.001);
  }
  // soc drop = I t / C = 3.2 A * 60 s / (0.6 Ah * 3600 s/h)
  const double expected_soc = 1.0 - 3.2 * 60.0 / (0.6 * 3600.0);
  EXPECT_NEAR(battery.state_of_charge(), expected_soc, 1e-9);
  EXPECT_NEAR(battery.current_a(), 3.2, 1e-12);
  // Linear interpolation between (0.90, 4.07) and (1.00, 4.20), minus I R.
  const double ocv = 4.07 + (expected_soc - 0.90) / 0.10 * (4.20 - 4.07);
  EXPECT_NEAR(battery.voltage_v(), ocv - 3.2 * 0.08, 1e-9);
}

TEST(BatteryModel, CurrentScalesWithThrustToThreeHalves) {
  const BatteryParams params = PlutoBattery();
  BatteryModel battery(params);
  battery.Step(4.0 * params.hover_thrust_n, 0.001);
  EXPECT_NEAR(battery.current_a(), 3.2 * 8.0, 1e-9);  // 4^1.5 = 8
}

TEST(BatteryModel, StateOfChargeClampsAtZero) {
  BatteryParams params = PlutoBattery();
  params.initial_state_of_charge = 0.0001;
  BatteryModel battery(params);
  for (int i = 0; i < 1000; ++i) {
    battery.Step(params.hover_thrust_n, 0.01);
  }
  EXPECT_DOUBLE_EQ(battery.state_of_charge(), 0.0);
  EXPECT_NEAR(battery.voltage_v(), 3.30 - 3.2 * 0.08, 1e-9);
}

TEST(BatteryModel, RejectsInvalidParameters) {
  BatteryParams params = PlutoBattery();
  params.open_circuit_voltage_table = {{0.0, 3.3}, {0.5, 3.8}};
  EXPECT_THROW(ValidateBatteryParams(params), std::invalid_argument);
  params = PlutoBattery();
  params.open_circuit_voltage_table = {{0.0, 3.3}, {0.5, 3.8}, {0.4, 3.9},
                                       {1.0, 4.2}};
  EXPECT_THROW(ValidateBatteryParams(params), std::invalid_argument);
  params = PlutoBattery();
  params.capacity_ah = 0.0;
  EXPECT_THROW(ValidateBatteryParams(params), std::invalid_argument);
  BatteryModel battery(PlutoBattery());
  EXPECT_THROW(battery.Step(-1.0, 0.001), std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
