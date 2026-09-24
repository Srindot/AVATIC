// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/propulsion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

PropulsionModel::PropulsionModel(const LegacyRotorParams& rotor,
                                 double arm_length_m,
                                 double rotor_inertia_kg_m2,
                                 const BatteryParams& battery)
    : mixer_(rotor, arm_length_m),
      max_rotor_speed_rad_s_(std::sqrt(rotor.speed_squared_max_rad2_s2)),
      spin_up_reaction_torque_(rotor.spin_up_reaction_torque),
      rotor_inertia_kg_m2_(rotor_inertia_kg_m2),
      actuators_{RotorActuator(rotor.time_constant_s),
                 RotorActuator(rotor.time_constant_s),
                 RotorActuator(rotor.time_constant_s),
                 RotorActuator(rotor.time_constant_s)},
      battery_(battery) {}

PropulsionOutput PropulsionModel::Step(
    const std::array<double, kRotorCount>& motor_duty, double timestep_s) {
  for (const double duty : motor_duty) {
    if (!IsFinite(duty) || duty < 0.0 || duty > 1.0) {
      throw std::invalid_argument("motor duty must be finite and in [0, 1]");
    }
  }
  const double voltage_ratio = std::sqrt(battery_.thrust_scale());
  std::array<double, kRotorCount> actual{};
  double spin_up_torque_n_m = 0.0;
  for (int i = 0; i < kRotorCount; ++i) {
    const double previous = actuators_[i].speed_rad_s();
    actual[i] = actuators_[i].Step(
        voltage_ratio * motor_duty[i] * max_rotor_speed_rad_s_, timestep_s);
    spin_up_torque_n_m += mixer_.geometry()[i].yaw_reaction_sign *
                          rotor_inertia_kg_m2_ * (actual[i] - previous) /
                          timestep_s;
  }

  PropulsionOutput output;
  output.rotors = mixer_.FromRotorSpeeds(actual);
  if (spin_up_reaction_torque_) {
    output.rotors.achieved.torque_frd_n_m.z() += spin_up_torque_n_m;
  }
  battery_.Step(std::max(output.rotors.achieved.thrust_n, 0.0), timestep_s);
  output.battery_voltage_v = battery_.voltage_v();
  output.battery_state_of_charge = battery_.state_of_charge();
  output.battery_current_a = battery_.current_a();
  return output;
}

}  // namespace pluto_x
