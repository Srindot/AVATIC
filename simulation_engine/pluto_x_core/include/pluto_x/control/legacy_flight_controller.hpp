// Copyright 2026 AVATIC contributors.
//
// The legacy kwad.cpp controller stack as a FlightController.
//
// Runs LegacyControllerStack every controller.period_s of simulation time on
// the full vehicle state (optionally corrupted by StateErrorModel), with the
// pilot command fixed by the configuration (pilot.initial_mode): the legacy
// stack has no RC interface, so RC frames are ignored. Its rotor speed
// commands are converted to motor duty as duty = w / w_max, the inverse of
// PropulsionModel's duty-to-speed map.

#ifndef PLUTO_X_CONTROL_LEGACY_FLIGHT_CONTROLLER_HPP_
#define PLUTO_X_CONTROL_LEGACY_FLIGHT_CONTROLLER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "pluto_x/common/backward_difference.hpp"
#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/flight_controller.hpp"
#include "pluto_x/control/legacy_controller_stack.hpp"
#include "pluto_x/sensors/state_error_model.hpp"

namespace pluto_x {

class LegacyFlightController : public FlightController {
 public:
  explicit LegacyFlightController(const LegacyStackConfig& config);

  std::string Name() const override { return "legacy_kwad_cascade"; }

  FlightControllerOutput Step(const FlightControllerInput& input) override;

  /// Most recent output of the legacy stack (setpoints, mixer), for logs.
  const ControllerOutput& last_stack_output() const { return last_output_; }

 private:
  double period_s_;
  std::int64_t period_ns_;
  double max_rotor_speed_rad_s_;
  PilotCommand pilot_;
  std::optional<std::int64_t> next_tick_ns_;

  LegacyControllerStack stack_;
  StateErrorModel state_error_;
  BackwardDifference3 angular_accel_estimator_;
  ControllerOutput last_output_;
  FlightControllerOutput last_command_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_LEGACY_FLIGHT_CONTROLLER_HPP_
