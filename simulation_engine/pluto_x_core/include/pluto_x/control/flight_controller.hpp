// Copyright 2026 AVATIC contributors.
//
// Flight-controller interface of the simulated vehicle.
//
// The simulator calls Step() once per physics step with the physical truth
// (state and acceleration) and the latest RC frame delivered by the command
// link. Each implementation decides internally when its control law runs and
// models its own sensors: the legacy stack runs every controller period on
// the (optionally corrupted) full state; MagisV2 runs the production firmware
// busy loop on simulated IMU/baro/magnetometer samples. The output is a motor
// duty cycle per rotor, in the mixer's rotor order (for magis_quad_x: M0..M3).
//
// Implementations: LegacyFlightController (this package) and
// pluto_x_magisv2::MagisFlightController (GPL package pluto_x_magisv2).

#ifndef PLUTO_X_CONTROL_FLIGHT_CONTROLLER_HPP_
#define PLUTO_X_CONTROL_FLIGHT_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "pluto_x/control/quad_mixer.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

inline constexpr int kRcChannelCount = 8;

/// One RC frame as carried by MSP_SET_RAW_RC: channels in microseconds,
/// AETR1234 order (roll, pitch, throttle, yaw, AUX1..AUX4).
struct RcFrame {
  std::array<std::uint16_t, kRcChannelCount> channels_us{};
  /// Increments with every frame received from the command source; lets a
  /// controller tell a new frame from a repeated one.
  std::uint64_t sequence{0};
};

/// Physical truth at the vehicle, used by the controller's sensor models.
struct VehicleTruth {
  VehicleStateNed state;
  /// Acceleration of the centre of mass in the NED world frame (including
  /// contact forces; an accelerometer at rest measures -g from this).
  Vector3 acceleration_ned_m_s2{Vector3::Zero()};
};

struct FlightControllerInput {
  std::int64_t time_ns{0};
  std::int64_t timestep_ns{0};
  VehicleTruth truth;
  /// Latest RC frame after the command-link latency; nullopt before the
  /// first frame arrives.
  std::optional<RcFrame> rc;
  /// Battery terminal voltage and current at the end of the previous step
  /// (what an on-board monitor would measure).
  double battery_voltage_v{0.0};
  double battery_current_a{0.0};
};

/// What the flight controller reports about itself: the information the
/// real Pluto X returns over its MSP link (MSP_STATUS, MSP_ATTITUDE,
/// MSP_ALTITUDE, MSP_ANALOG). Estimates, not truth. Physical conventions:
/// roll + = right side down, pitch + = NOSE UP, heading clockwise from north.
struct FlightControllerTelemetry {
  bool armed{false};
  bool ok_to_arm{false};
  bool calibrated{false};
  bool angle_mode{false};
  bool altitude_hold{false};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double heading_deg{0.0};  ///< [0, 360)
  double altitude_m{0.0};   ///< estimate relative to the arming/boot point
  double battery_v{0.0};
};

struct FlightControllerOutput {
  /// Commanded motor duty cycle in [0, 1] per rotor (mixer order).
  std::array<double, kRotorCount> motor_duty{};
  /// True if the control law executed during this step.
  bool control_step_ran{false};
  bool armed{false};
  /// False if the controller rejected its input or halted.
  bool healthy{true};
  FlightControllerTelemetry telemetry;
};

class FlightController {
 public:
  virtual ~FlightController() = default;

  /// Human-readable identifier for logs.
  virtual std::string Name() const = 0;

  /// Advances the controller to input.time_ns.
  virtual FlightControllerOutput Step(const FlightControllerInput& input) = 0;
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_FLIGHT_CONTROLLER_HPP_
