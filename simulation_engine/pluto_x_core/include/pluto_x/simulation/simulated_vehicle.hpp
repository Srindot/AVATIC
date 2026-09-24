// Copyright 2026 AVATIC contributors.
//
// Everything that happens to the vehicle in one physics step, independent of
// the simulator that integrates the rigid body. Used identically by the
// Gazebo plugin and by the offline reference simulation.
//
// Per call to Step(t, dt, truth, latest_rc):
//   1. wind           WindModel advances by dt
//   2. command link   the latest RC frame enters a DelayLine
//                     (latency.command_s); the delayed frame is delivered
//   3. controller     FlightController::Step (legacy stack or MagisV2),
//                     which runs its control law on its own schedule and
//                     returns motor duty
//   4. propulsion     PropulsionModel: voltage sag, motor lag -> actual
//                     rotor speeds and wrench, battery discharge
//   5. wrench         ComputeLegacyExternalWrench(true state, rotor wrench,
//                     wind) -> force (NED) and torque (FRD), gravity excluded
//
// Time is integer nanoseconds of simulation time; no wall clock is read. For
// a given configuration, controller and input sequence the output sequence
// is deterministic.

#ifndef PLUTO_X_SIMULATION_SIMULATED_VEHICLE_HPP_
#define PLUTO_X_SIMULATION_SIMULATED_VEHICLE_HPP_

#include <cstdint>
#include <memory>
#include <optional>

#include "pluto_x/common/delay_line.hpp"
#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/flight_controller.hpp"
#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "pluto_x/dynamics/propulsion_model.hpp"
#include "pluto_x/environment/wind_model.hpp"

namespace pluto_x {

struct SimulatedVehicleStep {
  /// Wrench to apply this physics step (gravity excluded).
  ExternalWrenchNedFrd wrench;
  FlightControllerOutput controller;
  /// Actual rotor state and battery after this step.
  PropulsionOutput propulsion;
  Vector3 wind_enu_m_s{Vector3::Zero()};
};

class SimulatedVehicle {
 public:
  /// Precondition: config validated; controller non-null (checked,
  /// std::invalid_argument).
  SimulatedVehicle(const LegacyStackConfig& config,
                   std::unique_ptr<FlightController> controller);

  /// Preconditions (checked, std::invalid_argument): timestep_ns > 0,
  /// sim_time_ns non-decreasing. latest_rc is the most recent frame from the
  /// command source (nullopt if none yet); frames are recognised by their
  /// sequence number. Invalid truth (NaN/Inf) yields a zero wrench.
  SimulatedVehicleStep Step(std::int64_t sim_time_ns, std::int64_t timestep_ns,
                            const VehicleTruth& truth,
                            const std::optional<RcFrame>& latest_rc);

  const FlightController& controller() const { return *controller_; }

 private:
  LegacyStackConfig config_;
  std::unique_ptr<FlightController> controller_;
  std::optional<std::int64_t> last_time_ns_;
  PropulsionModel propulsion_;
  WindModel wind_;
  DelayLine<std::optional<RcFrame>> rc_delay_;
  std::optional<std::uint64_t> last_pushed_sequence_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_SIMULATION_SIMULATED_VEHICLE_HPP_
