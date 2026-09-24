// Copyright 2026 AVATIC contributors.
//
// Wire layout of FlightControllerTelemetry on the simulator's gz-transport
// topic (gz.msgs.Double_V, default /pluto/fc_telemetry), shared by the
// Gazebo plugin (writer) and pluto_x_ros fc_link (reader). Booleans are
// 0.0 / 1.0. Index kSimTimeS carries the simulation time of the sample.

#ifndef PLUTO_X_CONTROL_TELEMETRY_WIRE_HPP_
#define PLUTO_X_CONTROL_TELEMETRY_WIRE_HPP_

#include <array>
#include <cstddef>

#include "pluto_x/control/flight_controller.hpp"

namespace pluto_x::telemetry_wire {

enum Index : std::size_t {
  kSimTimeS = 0,
  kArmed,
  kOkToArm,
  kCalibrated,
  kAngleMode,
  kAltitudeHold,
  kRollDeg,
  kPitchDeg,
  kHeadingDeg,
  kAltitudeM,
  kBatteryV,
  kHealthy,
  kCount,
};

using Frame = std::array<double, kCount>;

inline Frame Encode(double sim_time_s, const FlightControllerOutput& output) {
  const FlightControllerTelemetry& t = output.telemetry;
  Frame f{};
  f[kSimTimeS] = sim_time_s;
  f[kArmed] = t.armed ? 1.0 : 0.0;
  f[kOkToArm] = t.ok_to_arm ? 1.0 : 0.0;
  f[kCalibrated] = t.calibrated ? 1.0 : 0.0;
  f[kAngleMode] = t.angle_mode ? 1.0 : 0.0;
  f[kAltitudeHold] = t.altitude_hold ? 1.0 : 0.0;
  f[kRollDeg] = t.roll_deg;
  f[kPitchDeg] = t.pitch_deg;
  f[kHeadingDeg] = t.heading_deg;
  f[kAltitudeM] = t.altitude_m;
  f[kBatteryV] = t.battery_v;
  f[kHealthy] = output.healthy ? 1.0 : 0.0;
  return f;
}

}  // namespace pluto_x::telemetry_wire

#endif  // PLUTO_X_CONTROL_TELEMETRY_WIRE_HPP_
