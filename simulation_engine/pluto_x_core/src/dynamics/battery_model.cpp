// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/battery_model.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

constexpr double kSecondsPerHour = 3600.0;
/// Exponent of thrust in the current-draw model (momentum theory).
constexpr double kPowerThrustExponent = 1.5;

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument("battery: " + message);
  }
}

}  // namespace

void ValidateBatteryParams(const BatteryParams& p) {
  if (!p.enabled) {
    return;
  }
  Require(IsFinite(p.capacity_ah) && p.capacity_ah > 0.0, "capacity must be > 0");
  Require(IsFinite(p.initial_state_of_charge) &&
              p.initial_state_of_charge >= 0.0 &&
              p.initial_state_of_charge <= 1.0,
          "initial state of charge must be in [0, 1]");
  Require(IsFinite(p.internal_resistance_ohm) &&
              p.internal_resistance_ohm >= 0.0,
          "internal resistance must be >= 0");
  Require(IsFinite(p.reference_voltage_v) && p.reference_voltage_v > 0.0,
          "reference voltage must be > 0");
  Require(IsFinite(p.hover_current_a) && p.hover_current_a >= 0.0,
          "hover current must be >= 0");
  Require(IsFinite(p.hover_thrust_n) && p.hover_thrust_n > 0.0,
          "hover thrust must be > 0");
  const auto& table = p.open_circuit_voltage_table;
  Require(table.size() >= 2, "voltage table needs at least two points");
  Require(table.front().first == 0.0 && table.back().first == 1.0,
          "voltage table must span state of charge 0 to 1");
  for (std::size_t i = 0; i < table.size(); ++i) {
    Require(IsFinite(table[i].second) && table[i].second > 0.0,
            "voltage table voltages must be > 0");
    if (i > 0) {
      Require(table[i].first > table[i - 1].first,
              "voltage table state of charge must be strictly increasing");
    }
  }
}

BatteryModel::BatteryModel(const BatteryParams& params)
    : params_(params),
      state_of_charge_(params.initial_state_of_charge),
      voltage_v_(params.reference_voltage_v) {
  ValidateBatteryParams(params);
  if (params_.enabled) {
    voltage_v_ = OpenCircuitVoltage(state_of_charge_);
  }
}

void BatteryModel::Step(double total_thrust_n, double timestep_s) {
  if (!params_.enabled) {
    return;
  }
  if (!IsFinite(total_thrust_n) || total_thrust_n < 0.0) {
    throw std::invalid_argument("battery: thrust must be finite and >= 0");
  }
  if (!IsFinite(timestep_s) || !(timestep_s > 0.0)) {
    throw std::invalid_argument("battery: timestep must be > 0");
  }
  current_a_ = params_.hover_current_a *
               std::pow(total_thrust_n / params_.hover_thrust_n,
                        kPowerThrustExponent);
  state_of_charge_ -=
      current_a_ * timestep_s / (params_.capacity_ah * kSecondsPerHour);
  if (state_of_charge_ < 0.0) {
    state_of_charge_ = 0.0;
  }
  voltage_v_ = OpenCircuitVoltage(state_of_charge_) -
               params_.internal_resistance_ohm * current_a_;
  if (voltage_v_ < 0.0) {
    voltage_v_ = 0.0;
  }
}

double BatteryModel::thrust_scale() const {
  if (!params_.enabled) {
    return 1.0;
  }
  const double ratio = voltage_v_ / params_.reference_voltage_v;
  return ratio * ratio;
}

double BatteryModel::OpenCircuitVoltage(double soc) const {
  const auto& table = params_.open_circuit_voltage_table;
  for (std::size_t i = 1; i < table.size(); ++i) {
    if (soc <= table[i].first) {
      const double f =
          (soc - table[i - 1].first) / (table[i].first - table[i - 1].first);
      return table[i - 1].second + f * (table[i].second - table[i - 1].second);
    }
  }
  return table.back().second;
}

}  // namespace pluto_x
