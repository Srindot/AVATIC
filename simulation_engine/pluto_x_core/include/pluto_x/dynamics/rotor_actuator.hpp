// Copyright 2026 AVATIC contributors.
//
// First-order rotor speed response.
//
// ASSUMPTION: each rotor's speed follows its steady-state target through a
// first-order lag,  dw/dt = (w_target - w) / tau.
// The target is the commanded speed scaled by the supply voltage (see
// PropulsionModel). The update uses the exact discretisation
//   w <- w + (w_target - w) (1 - exp(-dt / tau)),
// which is stable for any dt. tau = 0 means an instantaneous response (the
// legacy behaviour).

#ifndef PLUTO_X_DYNAMICS_ROTOR_ACTUATOR_HPP_
#define PLUTO_X_DYNAMICS_ROTOR_ACTUATOR_HPP_

namespace pluto_x {

class RotorActuator {
 public:
  /// Throws std::invalid_argument if time_constant_s < 0 or not finite.
  explicit RotorActuator(double time_constant_s);

  /// Advances the rotor by timestep_s towards target_speed_rad_s and returns
  /// the new speed. Preconditions (checked): target finite and >= 0,
  /// timestep_s > 0.
  double Step(double target_speed_rad_s, double timestep_s);

  double speed_rad_s() const { return speed_rad_s_; }
  void Reset(double speed_rad_s);

 private:
  double time_constant_s_;
  double speed_rad_s_{0.0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_ROTOR_ACTUATOR_HPP_
