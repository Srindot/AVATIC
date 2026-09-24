// Copyright 2026 AVATIC contributors.
//
// Gazebo Harmonic system that flies the Pluto X model: glue between Gazebo
// and pluto_x::SimulatedVehicle.
//
// Per physics step (PreUpdate, simulation time only):
//   1. read the link's ENU pose/twist/acceleration from the ECM, convert to
//      NED/FRD (the acceleration is the physics engine's result of the
//      previous step, contact forces included: what an accelerometer feels)
//   2. pluto_x::SimulatedVehicle::Step() with the latest RC frame: wind,
//      command-link latency, flight controller (legacy stack or the MagisV2
//      firmware, selected by configuration), propulsion (motor lag,
//      battery), external wrench
//   3. apply the wrench with AddWorldWrench(); Gazebo integrates the rigid
//      body, including gravity and contact. The pose is never set directly.
//   4. publish battery state (10 Hz) and flight-controller telemetry
//      (50 Hz), both on simulation time
//
// RC input: gz.msgs.Int32_V with 8 channels in microseconds, AETR1234
// (published by pluto_x_ros fc_link from ROS pluto_x_interfaces/RcCommand).
// Frames arrive on a gz-transport thread and are handed to the physics
// thread under a mutex; each received frame gets a new sequence number.
//
// SDF parameters:
//   <config_file>        (required) YAML configuration
//   <flight_controller>  (optional) legacy | magisv2; overrides the YAML
//   <link_name>          (default base_link) link carrying all vehicle mass
//   <rc_topic>           (default /pluto/rc)
//   <battery_topic>      (default /pluto/battery) gz.msgs.BatteryState
//   <telemetry_topic>    (default /pluto/fc_telemetry) gz.msgs.Double_V,
//                        layout pluto_x/control/telemetry_wire.hpp
//
// Start-up consistency checks (failure disables the plugin with gzerr):
//   * world gravity == (0, 0, -vehicle.gravity_m_s2)
//   * link mass and diagonal inertia == configuration
//   * link inertial frame at the link origin (wrench is applied there)
//
// LICENSE NOTE: this plugin links pluto_x_magisv2 (GPL-3.0-or-later), so
// the plugin binary is covered by the GPL.

#ifndef PLUTO_X_GAZEBO_VEHICLE_SYSTEM_HPP_
#define PLUTO_X_GAZEBO_VEHICLE_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/int32_v.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/flight_controller.hpp"
#include "pluto_x/dynamics/enu_adapter.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"
#include "pluto_x/simulation/simulated_vehicle.hpp"

namespace pluto_x_gazebo {

class VehicleSystem : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

 private:
  bool LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf);
  bool ResolveLink(gz::sim::EntityComponentManager& ecm);
  bool CheckWorldGravity(const gz::sim::EntityComponentManager& ecm) const;
  bool CheckLinkInertial(const gz::sim::EntityComponentManager& ecm) const;
  bool CreateVehicle();

  struct SampledState {
    pluto_x::RigidBodyStateEnu simulator;  ///< as read from Gazebo
    pluto_x::VehicleTruth truth;           ///< same state, NED/FRD
  };

  /// Reads the link state; nullopt (and a one-time error log) if invalid.
  std::optional<SampledState> ReadState(
      const gz::sim::EntityComponentManager& ecm);
  void ApplyWrench(const pluto_x::ExternalWrenchNedFrd& wrench_ned,
                   const SampledState& state,
                   gz::sim::EntityComponentManager& ecm);
  void OnRc(const gz::msgs::Int32_V& message);
  std::optional<pluto_x::RcFrame> LatestRc();
  void ReportControllerEvents(const pluto_x::FlightControllerOutput& output);
  void PublishBattery(std::int64_t sim_time_ns,
                      const pluto_x::PropulsionOutput& propulsion);
  void PublishTelemetry(std::int64_t sim_time_ns,
                        const pluto_x::FlightControllerOutput& output);

  bool enabled_{false};
  std::string link_name_{"base_link"};
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  std::string rc_topic_{"/pluto/rc"};
  std::string battery_topic_{"/pluto/battery"};
  std::string telemetry_topic_{"/pluto/fc_telemetry"};

  pluto_x::LegacyStackConfig config_;
  std::unique_ptr<pluto_x::SimulatedVehicle> vehicle_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher battery_publisher_;
  gz::transport::Node::Publisher telemetry_publisher_;
  std::int64_t next_battery_publish_ns_{0};
  std::int64_t next_telemetry_publish_ns_{0};

  std::mutex rc_mutex_;
  std::optional<pluto_x::RcFrame> latest_rc_;  ///< guarded by rc_mutex_
  std::uint64_t rc_frames_received_{0};        ///< guarded by rc_mutex_
  bool reported_bad_rc_{false};                ///< guarded by rc_mutex_

  bool reported_first_rc_{false};
  bool reported_invalid_state_{false};
  bool reported_unhealthy_{false};
  std::optional<bool> reported_armed_;
};

}  // namespace pluto_x_gazebo

#endif  // PLUTO_X_GAZEBO_VEHICLE_SYSTEM_HPP_
