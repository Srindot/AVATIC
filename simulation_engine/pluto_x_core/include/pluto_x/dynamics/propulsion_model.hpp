// Copyright 2026 AVATIC contributors.
//
// Rotor speed command -> actual rotor speeds -> body wrench, with battery
// voltage sag.
//
// Per step:
//   1. voltage ratio r = V / V_reference (1 without a battery model)
//   2. target_i = r * duty_i * w_max, with w_max = sqrt(speed_squared_max)
//      the maximum rotor speed at the reference voltage.
//      ASSUMPTIONS: a brushed motor driven by PWM duty d spins at a steady
//      speed proportional to d (linear motor curve) and to the supply
//      voltage. Thrust therefore scales with (r d)^2. A measured
//      thrust-vs-duty curve (thrust stand) would replace the linear curve.
//   3. actual_i follows target_i through RotorActuator (first-order lag)
//   4. wrench = QuadMixer::FromRotorSpeeds(actual)
//      plus, if rotor.spin_up_reaction_torque, the reaction of the motors
//      accelerating their rotors about the yaw axis:
//        tau_z += sum(s_i * J_rotor * (w_i - w_i_prev) / dt)
//      (same sign as the drag reaction: speeding up a CCW rotor pushes the
//      body clockwise, i.e. +z_FRD)
//   5. the battery draws current for the resulting thrust
//
// With time constant 0 and the battery disabled, the output equals the
// controller's MixerOutput exactly (the legacy behaviour; tested).

#ifndef PLUTO_X_DYNAMICS_PROPULSION_MODEL_HPP_
#define PLUTO_X_DYNAMICS_PROPULSION_MODEL_HPP_

#include <array>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/quad_mixer.hpp"
#include "pluto_x/dynamics/battery_model.hpp"
#include "pluto_x/dynamics/rotor_actuator.hpp"

namespace pluto_x {

struct PropulsionOutput {
  MixerOutput rotors;  ///< actual speeds, achieved wrench, net rotor speed
  double battery_voltage_v{0.0};
  double battery_state_of_charge{1.0};
  double battery_current_a{0.0};
};

class PropulsionModel {
 public:
  PropulsionModel(const LegacyRotorParams& rotor, double arm_length_m,
                  double rotor_inertia_kg_m2, const BatteryParams& battery);

  /// Preconditions (checked, std::invalid_argument): duty finite and in
  /// [0, 1], timestep_s > 0.
  PropulsionOutput Step(const std::array<double, kRotorCount>& motor_duty,
                        double timestep_s);

  double max_rotor_speed_rad_s() const { return max_rotor_speed_rad_s_; }
  /// Battery terminal voltage/current after the most recent step.
  double battery_voltage_v() const { return battery_.voltage_v(); }
  double battery_current_a() const { return battery_.current_a(); }

 private:
  QuadMixer mixer_;
  double max_rotor_speed_rad_s_;
  bool spin_up_reaction_torque_;
  double rotor_inertia_kg_m2_;
  std::array<RotorActuator, kRotorCount> actuators_;
  BatteryModel battery_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_PROPULSION_MODEL_HPP_
