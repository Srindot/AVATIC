// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/rotor_actuator.hpp"

#include <cmath>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

RotorActuator::RotorActuator(double time_constant_s)
    : time_constant_s_(time_constant_s) {
  if (!IsFinite(time_constant_s) || time_constant_s < 0.0) {
    throw std::invalid_argument("rotor time constant must be >= 0");
  }
}

double RotorActuator::Step(double target_speed_rad_s, double timestep_s) {
  if (!IsFinite(target_speed_rad_s) || target_speed_rad_s < 0.0) {
    throw std::invalid_argument("rotor target speed must be finite and >= 0");
  }
  if (!IsFinite(timestep_s) || !(timestep_s > 0.0)) {
    throw std::invalid_argument("rotor timestep must be > 0");
  }
  if (time_constant_s_ == 0.0) {
    speed_rad_s_ = target_speed_rad_s;
  } else {
    const double blend = 1.0 - std::exp(-timestep_s / time_constant_s_);
    speed_rad_s_ += (target_speed_rad_s - speed_rad_s_) * blend;
  }
  return speed_rad_s_;
}

void RotorActuator::Reset(double speed_rad_s) {
  if (!IsFinite(speed_rad_s) || speed_rad_s < 0.0) {
    throw std::invalid_argument("rotor speed must be finite and >= 0");
  }
  speed_rad_s_ = speed_rad_s;
}

}  // namespace pluto_x
