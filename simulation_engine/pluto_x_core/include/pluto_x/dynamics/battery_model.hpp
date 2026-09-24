// Copyright 2026 AVATIC contributors.
//
// Single-cell LiPo battery with voltage sag.
//
// ASSUMPTIONS (all parameters in the YAML; values in
// docs/pluto_x_parameters.md):
//  * Terminal voltage   V = V_oc(soc) - R_internal * I
//    V_oc(soc) is piecewise-linear in the configured table.
//  * Current draw       I = I_hover * (T_total / T_hover)^(3/2)
//    (momentum theory: rotor power ~ thrust^1.5). No idle current.
//  * State of charge    d(soc)/dt = -I / capacity
//    soc is clamped at 0. The model does not cut off; a depleted battery
//    keeps its lowest table voltage.
//
// The effect on the vehicle, (V / V_reference)^2 on thrust, is applied by
// PropulsionModel.

#ifndef PLUTO_X_DYNAMICS_BATTERY_MODEL_HPP_
#define PLUTO_X_DYNAMICS_BATTERY_MODEL_HPP_

#include <utility>
#include <vector>

namespace pluto_x {

struct BatteryParams {
  bool enabled{false};
  double capacity_ah{0.0};
  double initial_state_of_charge{1.0};
  double internal_resistance_ohm{0.0};
  /// Voltage at which the rotor thrust parameters are defined.
  double reference_voltage_v{0.0};
  /// Current drawn when total thrust equals hover_thrust_n.
  double hover_current_a{0.0};
  double hover_thrust_n{0.0};
  /// (state of charge in [0, 1], open-circuit voltage), strictly increasing
  /// in state of charge, covering 0 and 1.
  std::vector<std::pair<double, double>> open_circuit_voltage_table;
};

/// Throws std::invalid_argument naming the problem if params are invalid.
/// Parameters of a disabled battery are not checked.
void ValidateBatteryParams(const BatteryParams& params);

class BatteryModel {
 public:
  explicit BatteryModel(const BatteryParams& params);

  /// Draws current for total_thrust_n over timestep_s. Preconditions
  /// (checked): total_thrust_n finite and >= 0, timestep_s > 0.
  /// A disabled battery ignores the call.
  void Step(double total_thrust_n, double timestep_s);

  /// Terminal voltage under the most recent load (reference voltage if
  /// disabled).
  double voltage_v() const { return voltage_v_; }
  double state_of_charge() const { return state_of_charge_; }
  double current_a() const { return current_a_; }
  /// (V / V_reference)^2; exactly 1 if disabled.
  double thrust_scale() const;

  bool enabled() const { return params_.enabled; }

 private:
  double OpenCircuitVoltage(double state_of_charge) const;

  BatteryParams params_;
  double state_of_charge_;
  double current_a_{0.0};
  double voltage_v_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_BATTERY_MODEL_HPP_
