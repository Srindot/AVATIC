// Copyright 2026 AVATIC contributors.
//
// Pilot command latched by the controller at each step.

#ifndef PLUTO_X_CONTROL_PILOT_COMMAND_HPP_
#define PLUTO_X_CONTROL_PILOT_COMMAND_HPP_

#include "pluto_x/config/legacy_params.hpp"

namespace pluto_x {

/// High-level command to the legacy controller stack. In kPositionHold the
/// thrust/roll/pitch fields are ignored (the position loop generates them).
struct PilotCommand {
  FlightMode mode{FlightMode::kManualAttitude};
  double thrust_n{0.0};
  double roll_setpoint_rad{0.0};
  double pitch_setpoint_rad{0.0};
};

bool IsFinite(const PilotCommand& command);

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_PILOT_COMMAND_HPP_
