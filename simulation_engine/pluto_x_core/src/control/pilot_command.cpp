// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/pilot_command.hpp"

#include "pluto_x/common/validation.hpp"

namespace pluto_x {

bool IsFinite(const PilotCommand& command) {
  return IsFinite(command.thrust_n) && IsFinite(command.roll_setpoint_rad) &&
         IsFinite(command.pitch_setpoint_rad);
}

}  // namespace pluto_x
