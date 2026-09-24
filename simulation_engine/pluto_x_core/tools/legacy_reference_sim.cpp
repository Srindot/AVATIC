// Copyright 2026 AVATIC contributors.
//
// Offline closed-loop run of SimulatedVehicle (controller, propulsion,
// battery, wind, sensor errors: exactly the code the Gazebo plugin runs) on
// the legacy reference integrator instead of Gazebo physics. No ROS.
//
// Purpose: a reference trajectory against which the Gazebo run is compared
// (compare_trajectories.py). The integrator steps at kPhysicsStepNs, the
// same step as worlds/legacy_flat.sdf, so both runs draw identical random
// sequences.
//
// Usage:
//   legacy_reference_sim <config.yaml> <duration_s> <output.csv>
//
// The run always uses position_hold from t = 0, with the vehicle at rest on
// the ground at the ENU origin facing north.
//
// CSV columns (ENU world for position, NED/FRD Euler for attitude):
//   t_s, x_enu_m, y_enu_m, z_enu_m, roll_frd_rad, pitch_frd_rad,
//   yaw_frd_rad, thrust_n, battery_v

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/config/config_loader.hpp"
#include "pluto_x/control/legacy_flight_controller.hpp"
#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "pluto_x/simulation/simulated_vehicle.hpp"

namespace {

constexpr int kExpectedArgumentCount = 4;
/// Must match <max_step_size> in pluto_x_gazebo/worlds/legacy_flat.sdf.
constexpr std::int64_t kPhysicsStepNs = 1000000;
/// One CSV row per this many physics steps (10 ms, like the odometry).
constexpr std::int64_t kOutputDecimation = 10;
constexpr double kNanosecondsPerSecond = 1e9;

int Run(const std::string& config_path, double duration_s,
        const std::string& output_path) {
  pluto_x::LegacyStackConfig config =
      pluto_x::LoadLegacyStackConfigFile(config_path);
  config.pilot.initial_mode = pluto_x::FlightMode::kPositionHold;
  pluto_x::SimulatedVehicle vehicle(
      config, std::make_unique<pluto_x::LegacyFlightController>(config));
  const pluto_x::LegacyReferenceIntegrator integrator(
      config.vehicle, config.reference_state_clamps);

  std::ofstream csv(output_path);
  if (!csv) {
    std::cerr << "cannot open output file " << output_path << "\n";
    return EXIT_FAILURE;
  }
  csv << "t_s,x_enu_m,y_enu_m,z_enu_m,roll_frd_rad,pitch_frd_rad,"
         "yaw_frd_rad,thrust_n,battery_v\n";
  csv << std::setprecision(9);

  pluto_x::VehicleTruth truth;
  pluto_x::VehicleStateNed& state = truth.state;

  const std::int64_t step_count = static_cast<std::int64_t>(
      duration_s * kNanosecondsPerSecond / static_cast<double>(kPhysicsStepNs));
  const double dt_s = static_cast<double>(kPhysicsStepNs) / kNanosecondsPerSecond;
  pluto_x::Vector3 previous_velocity_ned_m_s = state.velocity_ned_m_s;
  for (std::int64_t step = 0; step < step_count; ++step) {
    const std::int64_t t_ns = step * kPhysicsStepNs;
    const pluto_x::SimulatedVehicleStep out =
        vehicle.Step(t_ns, kPhysicsStepNs, truth, std::nullopt);
    if (!out.controller.healthy) {
      std::cerr << "controller rejected input at step " << step << "\n";
      return EXIT_FAILURE;
    }
    integrator.Step(state, out.propulsion.rotors.achieved,
                    out.propulsion.rotors.net_rotor_speed_rad_s, dt_s,
                    pluto_x::frames::SwapNedEnu(out.wind_enu_m_s),
                    out.propulsion.rotors.rotor_speed_sum_rad_s);
    // Acceleration after the ground clamp, as Gazebo reports it.
    truth.acceleration_ned_m_s2 =
        (state.velocity_ned_m_s - previous_velocity_ned_m_s) / dt_s;
    previous_velocity_ned_m_s = state.velocity_ned_m_s;

    if ((step + 1) % kOutputDecimation == 0) {
      const double t_s = static_cast<double>(t_ns + kPhysicsStepNs) /
                         kNanosecondsPerSecond;
      const pluto_x::Vector3 p = pluto_x::frames::SwapNedEnu(state.position_ned_m);
      csv << t_s << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
          << state.attitude.roll_rad << ',' << state.attitude.pitch_rad << ','
          << state.attitude.yaw_rad << ','
          << out.propulsion.rotors.achieved.thrust_n << ','
          << out.propulsion.battery_voltage_v << '\n';
    }
  }
  std::cout << "wrote " << step_count / kOutputDecimation << " samples to "
            << output_path << "\n";
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != kExpectedArgumentCount) {
    std::cerr << "usage: " << argv[0]
              << " <config.yaml> <duration_s> <output.csv>\n";
    return EXIT_FAILURE;
  }
  try {
    const double duration_s = std::stod(argv[2]);
    if (!(duration_s > 0.0)) {
      std::cerr << "duration_s must be > 0\n";
      return EXIT_FAILURE;
    }
    return Run(argv[1], duration_s, argv[3]);
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
