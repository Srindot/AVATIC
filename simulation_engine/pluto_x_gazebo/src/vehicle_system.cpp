// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later

#include "pluto_x_gazebo/vehicle_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <gz/common/Console.hh>
#include <gz/math/Inertial.hh>
#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Inertial.hh>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/config/config_loader.hpp"
#include "pluto_x/control/legacy_flight_controller.hpp"
#include "pluto_x/control/telemetry_wire.hpp"
#include "pluto_x_gazebo/gz_conversions.hpp"
#include "pluto_x_magisv2/magis_flight_controller.hpp"

namespace pluto_x_gazebo {
namespace {

/// Relative tolerance for comparing SDF mass properties and world gravity
/// with the YAML configuration. Both come from the same YAML via xacro, so
/// they should agree to printing precision.
constexpr double kConsistencyRelativeTolerance = 1e-6;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000;
constexpr std::int64_t kBatteryPublishPeriodNs = 100000000;   // 10 Hz
constexpr std::int64_t kTelemetryPublishPeriodNs = 20000000;  // 50 Hz

/// RC channel values accepted from the transport (MSP carries uint16; the
/// firmware constrains to its own limits).
constexpr std::int32_t kMinRcUs = 800;
constexpr std::int32_t kMaxRcUs = 2200;

bool NearlyEqual(double actual, double expected) {
  const double scale = std::max(std::abs(expected), 1.0);
  return std::abs(actual - expected) <= kConsistencyRelativeTolerance * scale;
}

std::int64_t ToNanoseconds(const std::chrono::steady_clock::duration& d) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

}  // namespace

void VehicleSystem::Configure(const gz::sim::Entity& entity,
                              const std::shared_ptr<const sdf::Element>& sdf,
                              gz::sim::EntityComponentManager& ecm,
                              gz::sim::EventManager& /*event_manager*/) {
  model_ = gz::sim::Model(entity);
  if (!model_.Valid(ecm)) {
    gzerr << "[pluto_x] VehicleSystem must be attached to a model; plugin "
             "disabled.\n";
    return;
  }
  if (!LoadConfiguration(sdf) || !ResolveLink(ecm) ||
      !CheckWorldGravity(ecm) || !CheckLinkInertial(ecm) || !CreateVehicle()) {
    gzerr << "[pluto_x] VehicleSystem disabled for model '"
          << model_.Name(ecm) << "'.\n";
    return;
  }

  battery_publisher_ = node_.Advertise<gz::msgs::BatteryState>(battery_topic_);
  telemetry_publisher_ = node_.Advertise<gz::msgs::Double_V>(telemetry_topic_);
  if (!node_.Subscribe(rc_topic_, &VehicleSystem::OnRc, this)) {
    gzerr << "[pluto_x] cannot subscribe to RC topic '" << rc_topic_
          << "'; plugin disabled.\n";
    return;
  }

  gzmsg << "[pluto_x] VehicleSystem configured for model '"
        << model_.Name(ecm) << "', link '" << link_name_ << "'.\n"
        << "[pluto_x] flight controller: " << vehicle_->controller().Name()
        << "; RC in '" << rc_topic_ << "', telemetry out '"
        << telemetry_topic_ << "', battery out '" << battery_topic_ << "'.\n"
        << "[pluto_x] Dynamics: Gazebo rigid-body physics + external wrench "
           "model; parameters are estimates, not validated.\n"
        << pluto_x::DescribeLegacyStackConfig(config_) << "\n";
  enabled_ = true;
}

bool VehicleSystem::LoadConfiguration(
    const std::shared_ptr<const sdf::Element>& sdf) {
  if (!sdf->HasElement("config_file")) {
    gzerr << "[pluto_x] missing required <config_file> parameter.\n";
    return false;
  }
  const std::string config_path = sdf->Get<std::string>("config_file");
  std::optional<std::string> flight_controller;
  if (sdf->HasElement("flight_controller")) {
    flight_controller = sdf->Get<std::string>("flight_controller");
    if (flight_controller->empty()) {
      flight_controller.reset();  // empty = use the YAML
    }
  }
  if (sdf->HasElement("link_name")) {
    link_name_ = sdf->Get<std::string>("link_name");
  }
  if (sdf->HasElement("rc_topic")) {
    rc_topic_ = sdf->Get<std::string>("rc_topic");
  }
  if (sdf->HasElement("battery_topic")) {
    battery_topic_ = sdf->Get<std::string>("battery_topic");
  }
  if (sdf->HasElement("telemetry_topic")) {
    telemetry_topic_ = sdf->Get<std::string>("telemetry_topic");
  }
  try {
    config_ = pluto_x::LoadLegacyStackConfigFile(config_path, flight_controller);
  } catch (const pluto_x::ConfigError& error) {
    gzerr << "[pluto_x] invalid configuration: " << error.what() << "\n";
    return false;
  }
  gzmsg << "[pluto_x] loaded configuration '" << config_path << "'"
        << (flight_controller ? " (flight_controller := " + *flight_controller + ")"
                              : std::string())
        << ".\n";
  return true;
}

bool VehicleSystem::CreateVehicle() {
  std::unique_ptr<pluto_x::FlightController> controller;
  try {
    switch (config_.flight_controller.type) {
      case pluto_x::FlightControllerType::kLegacy:
        controller = std::make_unique<pluto_x::LegacyFlightController>(config_);
        break;
      case pluto_x::FlightControllerType::kMagisV2:
        controller =
            std::make_unique<pluto_x_magisv2::MagisFlightController>(config_);
        break;
    }
    vehicle_ = std::make_unique<pluto_x::SimulatedVehicle>(
        config_, std::move(controller));
  } catch (const std::exception& error) {
    gzerr << "[pluto_x] cannot create the flight controller: " << error.what()
          << "\n";
    return false;
  }
  return true;
}

bool VehicleSystem::ResolveLink(gz::sim::EntityComponentManager& ecm) {
  link_ = gz::sim::Link(model_.LinkByName(ecm, link_name_));
  if (!link_.Valid(ecm)) {
    gzerr << "[pluto_x] link '" << link_name_ << "' not found.\n";
    return false;
  }
  // Velocity and acceleration components are not populated by default.
  link_.EnableVelocityChecks(ecm, true);
  link_.EnableAccelerationChecks(ecm, true);
  return true;
}

bool VehicleSystem::CheckWorldGravity(
    const gz::sim::EntityComponentManager& ecm) const {
  const gz::sim::World world(gz::sim::worldEntity(ecm));
  const std::optional<gz::math::Vector3d> gravity = world.Gravity(ecm);
  if (!gravity) {
    gzerr << "[pluto_x] world has no gravity component.\n";
    return false;
  }
  const double expected = -config_.vehicle.gravity_m_s2;
  if (!NearlyEqual(gravity->X(), 0.0) || !NearlyEqual(gravity->Y(), 0.0) ||
      !NearlyEqual(gravity->Z(), expected)) {
    gzerr << "[pluto_x] world gravity " << *gravity
          << " does not match configured (0 0 " << expected << ").\n";
    return false;
  }
  return true;
}

bool VehicleSystem::CheckLinkInertial(
    const gz::sim::EntityComponentManager& ecm) const {
  const auto* inertial_component =
      ecm.Component<gz::sim::components::Inertial>(link_.Entity());
  if (inertial_component == nullptr) {
    gzerr << "[pluto_x] link '" << link_name_ << "' has no inertial.\n";
    return false;
  }
  const gz::math::Inertiald& inertial = inertial_component->Data();
  const gz::math::MassMatrix3d& mass_matrix = inertial.MassMatrix();
  const gz::math::Vector3d diagonal = mass_matrix.DiagonalMoments();
  const gz::math::Vector3d off_diagonal = mass_matrix.OffDiagonalMoments();
  const pluto_x::Vector3& expected = config_.vehicle.inertia_diag_kg_m2;

  const bool ok = NearlyEqual(mass_matrix.Mass(), config_.vehicle.mass_kg) &&
                  NearlyEqual(diagonal.X(), expected.x()) &&
                  NearlyEqual(diagonal.Y(), expected.y()) &&
                  NearlyEqual(diagonal.Z(), expected.z()) &&
                  off_diagonal == gz::math::Vector3d::Zero;
  if (!ok) {
    gzerr << "[pluto_x] link mass/inertia (" << mass_matrix.Mass() << " kg, "
          << diagonal << " / " << off_diagonal
          << ") does not match configuration.\n";
    return false;
  }
  if (inertial.Pose() != gz::math::Pose3d::Zero) {
    gzerr << "[pluto_x] link inertial pose must be zero (the wrench is "
             "applied at the link origin); got "
          << inertial.Pose() << ".\n";
    return false;
  }
  return true;
}

void VehicleSystem::OnRc(const gz::msgs::Int32_V& message) {
  std::lock_guard<std::mutex> lock(rc_mutex_);
  bool valid = message.data_size() == pluto_x::kRcChannelCount;
  for (int i = 0; valid && i < message.data_size(); ++i) {
    valid = message.data(i) >= kMinRcUs && message.data(i) <= kMaxRcUs;
  }
  if (!valid) {
    if (!reported_bad_rc_) {
      gzwarn << "[pluto_x] ignoring RC message: need " << pluto_x::kRcChannelCount
             << " channels in [" << kMinRcUs << ", " << kMaxRcUs
             << "] us (reported once).\n";
      reported_bad_rc_ = true;
    }
    return;
  }
  pluto_x::RcFrame frame;
  for (int i = 0; i < pluto_x::kRcChannelCount; ++i) {
    frame.channels_us[i] = static_cast<std::uint16_t>(message.data(i));
  }
  frame.sequence = ++rc_frames_received_;
  latest_rc_ = frame;
}

std::optional<pluto_x::RcFrame> VehicleSystem::LatestRc() {
  std::lock_guard<std::mutex> lock(rc_mutex_);
  return latest_rc_;
}

void VehicleSystem::PreUpdate(const gz::sim::UpdateInfo& info,
                              gz::sim::EntityComponentManager& ecm) {
  if (!enabled_ || info.paused) {
    return;
  }
  const std::int64_t sim_time_ns = ToNanoseconds(info.simTime);
  const std::int64_t step_ns = ToNanoseconds(info.dt);
  if (step_ns <= 0) {
    return;
  }
  const std::optional<SampledState> state = ReadState(ecm);
  if (!state) {
    return;  // no wrench this step; error already reported
  }
  const std::optional<pluto_x::RcFrame> rc = LatestRc();
  if (rc && !reported_first_rc_) {
    gzmsg << "[pluto_x] first RC frame received at t = "
          << sim_time_ns * 1e-9 << " s.\n";
    reported_first_rc_ = true;
  }
  pluto_x::SimulatedVehicleStep step;
  try {
    step = vehicle_->Step(sim_time_ns, step_ns, state->truth, rc);
  } catch (const std::exception& error) {
    gzerr << "[pluto_x] simulation step failed: " << error.what()
          << "; plugin disabled.\n";
    enabled_ = false;
    return;
  }
  ApplyWrench(step.wrench, *state, ecm);
  ReportControllerEvents(step.controller);
  PublishBattery(sim_time_ns, step.propulsion);
  PublishTelemetry(sim_time_ns, step.controller);
}

std::optional<VehicleSystem::SampledState> VehicleSystem::ReadState(
    const gz::sim::EntityComponentManager& ecm) {
  SampledState sample;
  const gz::math::Pose3d pose = gz::sim::worldPose(link_.Entity(), ecm);
  sample.simulator.position_enu_m = ToEigen(pose.Pos());
  sample.simulator.orientation_enu_flu = ToEigen(pose.Rot());
  // Velocity/acceleration components are empty until the first physics
  // step; the vehicle starts at rest.
  sample.simulator.velocity_enu_m_s = ToEigen(
      link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero));
  sample.simulator.angular_velocity_enu_rad_s = ToEigen(
      link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero));
  const pluto_x::Vector3 acceleration_enu = ToEigen(
      link_.WorldLinearAcceleration(ecm).value_or(gz::math::Vector3d::Zero));
  try {
    sample.truth.state = pluto_x::ToVehicleStateNed(sample.simulator);
  } catch (const std::invalid_argument& error) {
    if (!reported_invalid_state_) {
      gzerr << "[pluto_x] invalid vehicle state from Gazebo ("
            << error.what() << "); no wrench applied.\n";
      reported_invalid_state_ = true;
    }
    return std::nullopt;
  }
  sample.truth.acceleration_ned_m_s2 =
      pluto_x::frames::SwapNedEnu(acceleration_enu);
  return sample;
}

void VehicleSystem::ApplyWrench(const pluto_x::ExternalWrenchNedFrd& wrench_ned,
                                const SampledState& state,
                                gz::sim::EntityComponentManager& ecm) {
  const pluto_x::WrenchEnuWorld wrench_enu = pluto_x::ToWrenchEnuWorld(
      wrench_ned, state.simulator.orientation_enu_flu);
  link_.AddWorldWrench(ecm, ToGz(wrench_enu.force_enu_n),
                       ToGz(wrench_enu.torque_enu_n_m));
}

void VehicleSystem::ReportControllerEvents(
    const pluto_x::FlightControllerOutput& output) {
  if (!output.healthy && !reported_unhealthy_) {
    gzerr << "[pluto_x] flight controller unhealthy (rejected input or "
             "firmware halted); motors off (reported once).\n";
    reported_unhealthy_ = true;
  }
  if (!reported_armed_ || *reported_armed_ != output.armed) {
    gzmsg << "[pluto_x] flight controller " << (output.armed ? "ARMED" : "disarmed")
          << ".\n";
    reported_armed_ = output.armed;
  }
}

void VehicleSystem::PublishBattery(std::int64_t sim_time_ns,
                                   const pluto_x::PropulsionOutput& propulsion) {
  if (!config_.battery.enabled || sim_time_ns < next_battery_publish_ns_) {
    return;
  }
  next_battery_publish_ns_ = sim_time_ns + kBatteryPublishPeriodNs;
  gz::msgs::BatteryState message;
  message.mutable_header()->mutable_stamp()->set_sec(
      static_cast<std::int32_t>(sim_time_ns / kNanosecondsPerSecond));
  message.mutable_header()->mutable_stamp()->set_nsec(
      static_cast<std::int32_t>(sim_time_ns % kNanosecondsPerSecond));
  message.set_voltage(propulsion.battery_voltage_v);
  message.set_current(propulsion.battery_current_a);
  message.set_capacity(config_.battery.capacity_ah);
  message.set_charge(config_.battery.capacity_ah *
                     propulsion.battery_state_of_charge);
  message.set_percentage(propulsion.battery_state_of_charge * 100.0);
  message.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
  battery_publisher_.Publish(message);
}

void VehicleSystem::PublishTelemetry(
    std::int64_t sim_time_ns, const pluto_x::FlightControllerOutput& output) {
  if (sim_time_ns < next_telemetry_publish_ns_) {
    return;
  }
  next_telemetry_publish_ns_ = sim_time_ns + kTelemetryPublishPeriodNs;
  const pluto_x::telemetry_wire::Frame frame = pluto_x::telemetry_wire::Encode(
      static_cast<double>(sim_time_ns) * 1e-9, output);
  gz::msgs::Double_V message;
  for (const double value : frame) {
    message.add_data(value);
  }
  telemetry_publisher_.Publish(message);
}

}  // namespace pluto_x_gazebo

GZ_ADD_PLUGIN(pluto_x_gazebo::VehicleSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
