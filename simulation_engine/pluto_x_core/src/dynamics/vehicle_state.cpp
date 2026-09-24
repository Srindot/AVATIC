// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/vehicle_state.hpp"

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

bool IsFinite(const VehicleStateNed& state) {
  return IsFinite(state.position_ned_m) && IsFinite(state.velocity_ned_m_s) &&
         IsFinite(state.attitude) && IsFinite(state.body_rate_frd_rad_s);
}

bool IsFinite(const BodyWrenchCommand& command) {
  return IsFinite(command.thrust_n) && IsFinite(command.torque_frd_n_m);
}

}  // namespace pluto_x
