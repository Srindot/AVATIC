// Copyright 2026 AVATIC contributors.

#ifndef PLUTO_X_CORE_TEST_TEST_HELPERS_HPP_
#define PLUTO_X_CORE_TEST_TEST_HELPERS_HPP_

#include <random>

#include "pluto_x/config/config_loader.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x::test {

/// The shipped legacy configuration (path injected by CMake).
inline LegacyStackConfig LoadDefaultConfig() {
  return LoadLegacyStackConfigFile(PLUTO_X_DEFAULT_CONFIG);
}

/// The estimated Pluto X configuration (path injected by CMake).
inline LegacyStackConfig LoadPlutoXConfig() {
  return LoadLegacyStackConfigFile(PLUTO_X_ESTIMATED_CONFIG);
}

/// Deterministic pseudo-random state generator for property tests.
class RandomStates {
 public:
  explicit RandomStates(unsigned seed) : engine_(seed) {}

  double Uniform(double lower, double upper) {
    return std::uniform_real_distribution<double>(lower, upper)(engine_);
  }

  Vector3 UniformVector(double magnitude) {
    return Vector3(Uniform(-magnitude, magnitude),
                   Uniform(-magnitude, magnitude),
                   Uniform(-magnitude, magnitude));
  }

  /// State with |roll|, |pitch| < 1.2 rad (away from gimbal lock).
  VehicleStateNed State() {
    VehicleStateNed state;
    state.position_ned_m = UniformVector(5.0);
    state.velocity_ned_m_s = UniformVector(3.0);
    state.attitude.roll_rad = Uniform(-1.2, 1.2);
    state.attitude.pitch_rad = Uniform(-1.2, 1.2);
    state.attitude.yaw_rad = Uniform(-3.1, 3.1);
    state.body_rate_frd_rad_s = UniformVector(2.0);
    return state;
  }

 private:
  std::mt19937 engine_;
};

}  // namespace pluto_x::test

#endif  // PLUTO_X_CORE_TEST_TEST_HELPERS_HPP_
