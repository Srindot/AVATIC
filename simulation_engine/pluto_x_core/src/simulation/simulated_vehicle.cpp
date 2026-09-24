// Copyright 2026 AVATIC contributors.

#include "pluto_x/simulation/simulated_vehicle.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;

std::int64_t ToNanoseconds(double seconds) {
  return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

}  // namespace

SimulatedVehicle::SimulatedVehicle(const LegacyStackConfig& config,
                                   std::unique_ptr<FlightController> controller)
    : config_(config),
      controller_(std::move(controller)),
      propulsion_(config.rotor, config.vehicle.arm_length_m,
                  config.vehicle.rotor_inertia_kg_m2, config.battery),
      wind_(config.wind),
      rc_delay_(ToNanoseconds(config.latency.command_s), std::nullopt) {
  if (!controller_) {
    throw std::invalid_argument("SimulatedVehicle requires a flight controller");
  }
}

SimulatedVehicleStep SimulatedVehicle::Step(
    std::int64_t sim_time_ns, std::int64_t timestep_ns,
    const VehicleTruth& truth, const std::optional<RcFrame>& latest_rc) {
  if (timestep_ns <= 0) {
    throw std::invalid_argument("SimulatedVehicle timestep must be > 0");
  }
  if (last_time_ns_ && sim_time_ns < *last_time_ns_) {
    throw std::invalid_argument("SimulatedVehicle time must not decrease");
  }
  last_time_ns_ = sim_time_ns;
  const double timestep_s =
      static_cast<double>(timestep_ns) / kNanosecondsPerSecond;

  SimulatedVehicleStep step;
  step.wind_enu_m_s = wind_.Step(timestep_s);

  if (latest_rc && latest_rc->sequence != last_pushed_sequence_) {
    rc_delay_.Push(sim_time_ns, latest_rc);
    last_pushed_sequence_ = latest_rc->sequence;
  }

  FlightControllerInput input;
  input.time_ns = sim_time_ns;
  input.timestep_ns = timestep_ns;
  input.truth = truth;
  input.rc = rc_delay_.Sample(sim_time_ns);
  input.battery_voltage_v = propulsion_.battery_voltage_v();
  input.battery_current_a = propulsion_.battery_current_a();
  step.controller = controller_->Step(input);

  step.propulsion = propulsion_.Step(step.controller.motor_duty, timestep_s);
  if (IsFinite(truth.state)) {
    step.wrench = ComputeLegacyExternalWrench(
        truth.state, step.propulsion.rotors.achieved,
        step.propulsion.rotors.net_rotor_speed_rad_s, config_.vehicle,
        frames::SwapNedEnu(step.wind_enu_m_s),
        step.propulsion.rotors.rotor_speed_sum_rad_s);
  }
  return step;
}

}  // namespace pluto_x
