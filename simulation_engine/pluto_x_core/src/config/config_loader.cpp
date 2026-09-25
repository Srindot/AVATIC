// Copyright 2026 AVATIC contributors.

#include "pluto_x/config/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

#include "pluto_x/common/validation.hpp"
#include "pluto_x/sensors/baro_model.hpp"
#include "pluto_x/sensors/imu_model.hpp"

namespace pluto_x {
namespace {

std::string JoinPath(const std::string& parent, const std::string& key) {
  return parent.empty() ? key : parent + "." + key;
}

/// Returns the child map/scalar `key` of `node` or throws naming the path.
YAML::Node RequireChild(const YAML::Node& node, const std::string& key,
                        const std::string& parent_path) {
  const std::string path = JoinPath(parent_path, key);
  if (!node.IsMap()) {
    throw ConfigError("'" + parent_path + "' must be a map");
  }
  const YAML::Node child = node[key];
  if (!child) {
    throw ConfigError("missing required parameter '" + path + "'");
  }
  return child;
}

double ReadDouble(const YAML::Node& node, const std::string& key,
                  const std::string& parent_path) {
  const std::string path = JoinPath(parent_path, key);
  const YAML::Node child = RequireChild(node, key, parent_path);
  double value = 0.0;
  try {
    value = child.as<double>();
  } catch (const YAML::Exception&) {
    throw ConfigError("parameter '" + path + "' must be a number");
  }
  if (!IsFinite(value)) {
    throw ConfigError("parameter '" + path + "' must be finite");
  }
  return value;
}

double ReadPositive(const YAML::Node& node, const std::string& key,
                    const std::string& parent_path) {
  const double value = ReadDouble(node, key, parent_path);
  if (!(value > 0.0)) {
    throw ConfigError("parameter '" + JoinPath(parent_path, key) +
                      "' must be > 0");
  }
  return value;
}

double ReadNonNegative(const YAML::Node& node, const std::string& key,
                       const std::string& parent_path) {
  const double value = ReadDouble(node, key, parent_path);
  if (!(value >= 0.0)) {
    throw ConfigError("parameter '" + JoinPath(parent_path, key) +
                      "' must be >= 0");
  }
  return value;
}

std::string ReadString(const YAML::Node& node, const std::string& key,
                       const std::string& parent_path) {
  const YAML::Node child = RequireChild(node, key, parent_path);
  try {
    return child.as<std::string>();
  } catch (const YAML::Exception&) {
    throw ConfigError("parameter '" + JoinPath(parent_path, key) +
                      "' must be a string");
  }
}

bool ReadBool(const YAML::Node& node, const std::string& key,
              const std::string& parent_path) {
  const YAML::Node child = RequireChild(node, key, parent_path);
  try {
    return child.as<bool>();
  } catch (const YAML::Exception&) {
    throw ConfigError("parameter '" + JoinPath(parent_path, key) +
                      "' must be true or false");
  }
}

/// Reads a {first, second, third}-keyed map into a Vector3, requiring each
/// component to be >= 0 when `non_negative` is set.
Vector3 ReadVector3(const YAML::Node& node, const std::string& key,
                    const std::string& parent_path, const char* first,
                    const char* second, const char* third,
                    bool non_negative) {
  const std::string path = JoinPath(parent_path, key);
  const YAML::Node child = RequireChild(node, key, parent_path);
  if (non_negative) {
    return Vector3(ReadNonNegative(child, first, path),
                   ReadNonNegative(child, second, path),
                   ReadNonNegative(child, third, path));
  }
  return Vector3(ReadDouble(child, first, path),
                 ReadDouble(child, second, path),
                 ReadDouble(child, third, path));
}

LegacyPidGains ReadPidGains(const YAML::Node& node, const std::string& key,
                            const std::string& parent_path) {
  const std::string path = JoinPath(parent_path, key);
  const YAML::Node child = RequireChild(node, key, parent_path);
  LegacyPidGains gains;
  gains.kp = ReadDouble(child, "kp", path);
  gains.ki = ReadDouble(child, "ki", path);
  gains.kd = ReadDouble(child, "kd", path);
  gains.integral_window = ReadNonNegative(child, "integral_window", path);
  return gains;
}

FlightMode ParseFlightMode(const std::string& text, const std::string& path) {
  if (text == "manual_attitude") {
    return FlightMode::kManualAttitude;
  }
  if (text == "position_hold") {
    return FlightMode::kPositionHold;
  }
  throw ConfigError("parameter '" + path +
                    "' must be 'manual_attitude' or 'position_hold'");
}

RotorLayout ParseRotorLayout(const std::string& text,
                             const std::string& path) {
  if (text == "legacy_plus") {
    return RotorLayout::kLegacyPlus;
  }
  if (text == "magis_quad_x") {
    return RotorLayout::kMagisQuadX;
  }
  throw ConfigError("parameter '" + path +
                    "' must be 'legacy_plus' or 'magis_quad_x'");
}

const char* RotorLayoutName(RotorLayout layout) {
  return layout == RotorLayout::kMagisQuadX ? "magis_quad_x" : "legacy_plus";
}

PositionErrorFrame ParsePositionErrorFrame(const std::string& text,
                                           const std::string& path) {
  if (text == "legacy_full_attitude") {
    return PositionErrorFrame::kLegacyFullAttitude;
  }
  if (text == "yaw_only") {
    return PositionErrorFrame::kYawOnly;
  }
  throw ConfigError("parameter '" + path +
                    "' must be 'legacy_full_attitude' or 'yaw_only'");
}

const char* PositionErrorFrameName(PositionErrorFrame frame) {
  switch (frame) {
    case PositionErrorFrame::kLegacyFullAttitude:
      return "legacy_full_attitude";
    case PositionErrorFrame::kYawOnly:
      return "yaw_only";
  }
  return "unknown";
}

const char* FlightModeName(FlightMode mode) {
  switch (mode) {
    case FlightMode::kManualAttitude:
      return "manual_attitude";
    case FlightMode::kPositionHold:
      return "position_hold";
  }
  return "unknown";
}

LegacyVehicleParams ReadVehicle(const YAML::Node& root) {
  const std::string path = "vehicle";
  const YAML::Node node = RequireChild(root, path, "");
  LegacyVehicleParams vehicle;
  vehicle.mass_kg = ReadPositive(node, "mass_kg", path);
  vehicle.gravity_m_s2 = ReadPositive(node, "gravity_m_s2", path);
  vehicle.inertia_diag_kg_m2 =
      ReadVector3(node, "inertia_kg_m2", path, "xx", "yy", "zz", true);
  vehicle.rotor_inertia_kg_m2 =
      ReadNonNegative(node, "rotor_inertia_kg_m2", path);
  vehicle.arm_length_m = ReadPositive(node, "arm_length_m", path);
  vehicle.legacy_double_arm_torque =
      ReadBool(node, "legacy_double_arm_torque", path);
  vehicle.legacy_gyroscopic_form =
      ReadBool(node, "legacy_gyroscopic_form", path);
  const std::string drag_path = JoinPath(path, "drag");
  const YAML::Node drag = RequireChild(node, "drag", path);
  const std::string model = ReadString(drag, "model", drag_path);
  if (model == "legacy_linear") {
    vehicle.drag_model = DragModel::kLegacyLinear;
  } else if (model == "rotor_and_body") {
    vehicle.drag_model = DragModel::kRotorAndBody;
  } else {
    throw ConfigError("parameter '" + JoinPath(drag_path, "model") +
                      "' must be 'legacy_linear' or 'rotor_and_body'");
  }
  vehicle.linear_drag_ned_n_s_m = ReadVector3(
      drag, "linear_ned_n_s_m", drag_path, "north", "east", "down", true);
  vehicle.rotor_drag_coefficient_n_s_m_rad =
      ReadNonNegative(drag, "rotor_drag_coefficient_n_s_m_rad", drag_path);
  vehicle.body_drag_area_frd_m2 = ReadVector3(
      drag, "body_drag_area_frd_m2", drag_path, "x", "y", "z", true);
  vehicle.air_density_kg_m3 =
      ReadNonNegative(drag, "air_density_kg_m3", drag_path);
  return vehicle;
}

LegacyRotorParams ReadRotor(const YAML::Node& root) {
  const std::string path = "rotor";
  const YAML::Node node = RequireChild(root, path, "");
  LegacyRotorParams rotor;
  rotor.layout = ParseRotorLayout(ReadString(node, "layout", path),
                                  JoinPath(path, "layout"));
  rotor.time_constant_s = ReadNonNegative(node, "time_constant_s", path);
  rotor.spin_up_reaction_torque =
      ReadBool(node, "spin_up_reaction_torque", path);
  rotor.thrust_coefficient_n_s2 =
      ReadPositive(node, "thrust_coefficient_n_s2", path);
  rotor.yaw_drag_coefficient_n_m_s2 =
      ReadPositive(node, "yaw_drag_coefficient_n_m_s2", path);
  rotor.speed_squared_min_rad2_s2 =
      ReadNonNegative(node, "speed_squared_min_rad2_s2", path);
  rotor.speed_squared_max_rad2_s2 =
      ReadPositive(node, "speed_squared_max_rad2_s2", path);
  return rotor;
}

LegacyActuationLimits ReadActuationLimits(const YAML::Node& root) {
  const std::string path = "actuation_limits";
  const YAML::Node node = RequireChild(root, path, "");
  const std::string thrust_path = JoinPath(path, "thrust_n");
  const YAML::Node thrust = RequireChild(node, "thrust_n", path);
  LegacyActuationLimits limits;
  limits.thrust_min_n = ReadNonNegative(thrust, "min", thrust_path);
  limits.thrust_max_n = ReadPositive(thrust, "max", thrust_path);
  limits.torque_limit_n_m =
      ReadVector3(node, "torque_n_m", path, "roll", "pitch", "yaw", true);
  return limits;
}

LegacyControllerParams ReadController(const YAML::Node& root) {
  const std::string path = "controller";
  const YAML::Node node = RequireChild(root, path, "");
  LegacyControllerParams controller;
  controller.period_s = ReadPositive(node, "period_s", path);

  const std::string pos_path = JoinPath(path, "position");
  const YAML::Node pos = RequireChild(node, "position", path);
  controller.position.error_frame =
      ParsePositionErrorFrame(ReadString(pos, "error_frame", pos_path),
                              JoinPath(pos_path, "error_frame"));
  controller.position.north = ReadPidGains(pos, "north", pos_path);
  controller.position.east = ReadPidGains(pos, "east", pos_path);
  controller.position.down = ReadPidGains(pos, "down", pos_path);
  controller.position.roll_limit_rad =
      ReadPositive(pos, "roll_limit_rad", pos_path);
  controller.position.pitch_limit_rad =
      ReadPositive(pos, "pitch_limit_rad", pos_path);

  const std::string att_path = JoinPath(path, "attitude");
  const YAML::Node att = RequireChild(node, "attitude", path);
  controller.attitude.roll = ReadPidGains(att, "roll", att_path);
  controller.attitude.pitch = ReadPidGains(att, "pitch", att_path);
  controller.attitude.yaw = ReadPidGains(att, "yaw", att_path);
  controller.attitude.body_rate_limit_rad_s = ReadVector3(
      att, "body_rate_limit_rad_s", att_path, "roll", "pitch", "yaw", true);

  const std::string rate_path = JoinPath(path, "rate");
  const YAML::Node rate = RequireChild(node, "rate", path);
  controller.rate.roll = ReadPidGains(rate, "roll", rate_path);
  controller.rate.pitch = ReadPidGains(rate, "pitch", rate_path);
  controller.rate.yaw = ReadPidGains(rate, "yaw", rate_path);
  return controller;
}

LegacyPilotParams ReadPilot(const YAML::Node& root) {
  const std::string path = "pilot";
  const YAML::Node node = RequireChild(root, path, "");
  LegacyPilotParams pilot;
  pilot.initial_mode = ParseFlightMode(
      ReadString(node, "initial_mode", path), JoinPath(path, "initial_mode"));
  pilot.hover_thrust_n = ReadNonNegative(node, "hover_thrust_n", path);
  pilot.hold_initial_heading = ReadBool(node, "hold_initial_heading", path);
  pilot.position_hold_target_enu_m = ReadVector3(
      node, "position_hold_target_enu_m", path, "x", "y", "z", false);

  return pilot;
}

LegacyStateClampParams ReadStateClamps(const YAML::Node& root) {
  const std::string path = "reference_state_clamps";
  const YAML::Node node = RequireChild(root, path, "");
  LegacyStateClampParams clamps;
  clamps.body_rate_limit_rad_s = ReadVector3(node, "body_rate_rad_s", path,
                                             "roll", "pitch", "yaw", true);
  const Vector3 attitude =
      ReadVector3(node, "attitude_rad", path, "roll", "pitch", "yaw", true);
  clamps.attitude_limit_rad.roll_rad = attitude.x();
  clamps.attitude_limit_rad.pitch_rad = attitude.y();
  clamps.attitude_limit_rad.yaw_rad = attitude.z();
  return clamps;
}

std::uint64_t ReadSeed(const YAML::Node& node, const std::string& key,
                       const std::string& parent_path) {
  const YAML::Node child = RequireChild(node, key, parent_path);
  try {
    return child.as<std::uint64_t>();
  } catch (const YAML::Exception&) {
    throw ConfigError("parameter '" + JoinPath(parent_path, key) +
                      "' must be a non-negative integer");
  }
}

BatteryParams ReadBattery(const YAML::Node& root,
                          const LegacyVehicleParams& vehicle) {
  const std::string path = "battery";
  const YAML::Node node = RequireChild(root, path, "");
  BatteryParams battery;
  battery.enabled = ReadBool(node, "enabled", path);
  constexpr double kMilliampHoursPerAmpHour = 1000.0;
  battery.capacity_ah =
      ReadPositive(node, "capacity_mah", path) / kMilliampHoursPerAmpHour;
  battery.initial_state_of_charge =
      ReadNonNegative(node, "initial_state_of_charge", path);
  battery.internal_resistance_ohm =
      ReadNonNegative(node, "internal_resistance_ohm", path);
  battery.reference_voltage_v = ReadPositive(node, "reference_voltage_v", path);
  battery.hover_current_a = ReadNonNegative(node, "hover_current_a", path);
  battery.hover_thrust_n = vehicle.mass_kg * vehicle.gravity_m_s2;

  const std::string table_path = JoinPath(path, "open_circuit_voltage");
  const YAML::Node table = RequireChild(node, "open_circuit_voltage", path);
  if (!table.IsSequence()) {
    throw ConfigError("'" + table_path +
                      "' must be a list of [state_of_charge, volts] pairs");
  }
  for (std::size_t i = 0; i < table.size(); ++i) {
    const YAML::Node entry = table[i];
    if (!entry.IsSequence() || entry.size() != 2) {
      throw ConfigError("'" + table_path + "' entry " + std::to_string(i) +
                        " must be [state_of_charge, volts]");
    }
    double soc = 0.0;
    double volts = 0.0;
    try {
      soc = entry[0].as<double>();
      volts = entry[1].as<double>();
    } catch (const YAML::Exception&) {
      throw ConfigError("'" + table_path + "' entry " + std::to_string(i) +
                        " must contain numbers");
    }
    battery.open_circuit_voltage_table.emplace_back(soc, volts);
  }
  try {
    ValidateBatteryParams(battery);
  } catch (const std::invalid_argument& error) {
    throw ConfigError(error.what());
  }
  return battery;
}

LatencyParams ReadLatency(const YAML::Node& root) {
  const std::string path = "latency";
  const YAML::Node node = RequireChild(root, path, "");
  LatencyParams latency;
  latency.command_s = ReadNonNegative(node, "command_s", path);
  return latency;
}

StateErrorParams ReadStateError(const YAML::Node& root) {
  const std::string path = "state_error";
  const YAML::Node node = RequireChild(root, path, "");
  StateErrorParams e;
  e.enabled = ReadBool(node, "enabled", path);
  e.seed = ReadSeed(node, "seed", path);
  e.attitude_bias_std_rad = ReadNonNegative(node, "attitude_bias_std_rad", path);
  e.attitude_noise_std_rad =
      ReadNonNegative(node, "attitude_noise_std_rad", path);
  e.gyro_bias_std_rad_s = ReadNonNegative(node, "gyro_bias_std_rad_s", path);
  e.gyro_noise_std_rad_s = ReadNonNegative(node, "gyro_noise_std_rad_s", path);
  return e;
}

WindParams ReadWind(const YAML::Node& root) {
  const std::string path = "wind";
  const YAML::Node node = RequireChild(root, path, "");
  WindParams wind;
  wind.enabled = ReadBool(node, "enabled", path);
  wind.seed = ReadSeed(node, "seed", path);
  wind.mean_enu_m_s = ReadVector3(node, "mean_enu_m_s", path, "x", "y", "z",
                                  false);
  wind.gust_std_m_s = ReadNonNegative(node, "gust_std_m_s", path);
  wind.gust_time_constant_s = ReadPositive(node, "gust_time_constant_s", path);
  return wind;
}

FlightControllerType ParseFlightControllerType(const std::string& text,
                                               const std::string& path) {
  if (text == "legacy") return FlightControllerType::kLegacy;
  if (text == "magisv2") return FlightControllerType::kMagisV2;
  throw ConfigError("parameter '" + path + "' must be legacy or magisv2, got '" +
                    text + "'");
}

const char* FlightControllerTypeName(FlightControllerType type) {
  return type == FlightControllerType::kMagisV2 ? "magisv2" : "legacy";
}

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
/// Standard gravity: accelerometer sensitivities are specified per g0.
constexpr double kStandardGravityM_s2 = 9.80665;
constexpr double kMilliG = 1e-3 * kStandardGravityM_s2;

ImuParams ReadMagisImu(const YAML::Node& parent, const std::string& parent_path) {
  const std::string path = JoinPath(parent_path, "imu");
  const YAML::Node node = RequireChild(parent, "imu", parent_path);
  ImuParams imu;
  imu.seed = ReadSeed(node, "seed", path);
  imu.gyro_counts_per_rad_s =
      ReadPositive(node, "gyro_counts_per_deg_s", path) / kDegToRad;
  imu.accel_counts_per_m_s2 =
      ReadPositive(node, "accel_counts_per_g", path) / kStandardGravityM_s2;
  imu.gyro_range_rad_s = ReadPositive(node, "gyro_range_deg_s", path) * kDegToRad;
  imu.accel_range_m_s2 =
      ReadPositive(node, "accel_range_g", path) * kStandardGravityM_s2;
  imu.gyro_noise_std_rad_s =
      ReadNonNegative(node, "gyro_noise_std_deg_s", path) * kDegToRad;
  imu.accel_noise_std_m_s2 =
      ReadNonNegative(node, "accel_noise_std_mg", path) * kMilliG;
  imu.gyro_bias_std_rad_s =
      ReadNonNegative(node, "gyro_bias_std_deg_s", path) * kDegToRad;
  imu.accel_bias_std_m_s2 =
      ReadNonNegative(node, "accel_bias_std_mg", path) * kMilliG;
  // FRD body -> FLU sensor frame of the firmware.
  imu.sensor_from_body = Vector3(1.0, -1.0, -1.0).asDiagonal();
  try {
    ValidateImuParams(imu);
  } catch (const std::invalid_argument& error) {
    throw ConfigError(path + ": " + error.what());
  }
  return imu;
}

BaroParams ReadMagisBaro(const YAML::Node& parent,
                         const std::string& parent_path) {
  const std::string path = JoinPath(parent_path, "baro");
  const YAML::Node node = RequireChild(parent, "baro", parent_path);
  BaroParams baro;
  baro.seed = ReadSeed(node, "seed", path);
  baro.sea_level_pressure_pa = ReadPositive(node, "sea_level_pressure_pa", path);
  baro.ground_altitude_msl_m = ReadDouble(node, "ground_altitude_msl_m", path);
  baro.temperature_c = ReadDouble(node, "temperature_c", path);
  baro.noise_std_pa = ReadNonNegative(node, "noise_std_pa", path);
  try {
    BaroModel check(baro);
  } catch (const std::invalid_argument& error) {
    throw ConfigError(path + ": " + error.what());
  }
  return baro;
}

FlightControllerParams ReadFlightController(const YAML::Node& root) {
  const std::string path = "flight_controller";
  const YAML::Node node = RequireChild(root, path, "");
  FlightControllerParams fc;
  fc.type = ParseFlightControllerType(ReadString(node, "type", path),
                                      JoinPath(path, "type"));
  if (fc.type != FlightControllerType::kMagisV2) {
    return fc;
  }
  const std::string mpath = JoinPath(path, "magisv2");
  const YAML::Node m = RequireChild(node, "magisv2", path);
  MagisV2Params& magis = fc.magisv2;
  const double loop_us = ReadPositive(m, "busy_loop_period_us", mpath);
  if (loop_us != std::floor(loop_us) || loop_us > 3500.0) {
    // loop() must run at least once per firmware looptime (3500 us).
    throw ConfigError("parameter '" + JoinPath(mpath, "busy_loop_period_us") +
                      "' must be an integer in [1, 3500]");
  }
  magis.busy_loop_period_us = static_cast<std::uint32_t>(loop_us);
  magis.preload_mag_calibration =
      ReadBool(m, "preload_mag_calibration", mpath);
  magis.imu = ReadMagisImu(m, mpath);
  magis.baro = ReadMagisBaro(m, mpath);
  magis.magnetic_field_ned_ut = ReadVector3(m, "magnetic_field_ned_ut", mpath,
                                            "north", "east", "down", false);
  if (!(magis.magnetic_field_ned_ut.norm() > 0.0)) {
    throw ConfigError("parameter '" + JoinPath(mpath, "magnetic_field_ned_ut") +
                      "' must be nonzero");
  }
  magis.mag_counts_per_ut = ReadPositive(m, "mag_counts_per_ut", mpath);
  return fc;
}

LegacyStackConfig ParseRoot(const YAML::Node& root) {
  if (!root.IsMap()) {
    throw ConfigError("configuration root must be a YAML map");
  }
  LegacyStackConfig config;
  config.vehicle = ReadVehicle(root);
  config.rotor = ReadRotor(root);
  config.actuation_limits = ReadActuationLimits(root);
  config.controller = ReadController(root);
  config.pilot = ReadPilot(root);
  config.reference_state_clamps = ReadStateClamps(root);
  config.battery = ReadBattery(root, config.vehicle);
  config.latency = ReadLatency(root);
  config.state_error = ReadStateError(root);
  config.wind = ReadWind(root);
  config.flight_controller = ReadFlightController(root);
  ValidateLegacyStackConfig(config);
  return config;
}

void RequireOrderedConfig(double lower, double upper, const std::string& what) {
  if (!(lower <= upper)) {
    throw ConfigError(what + ": min must not exceed max");
  }
}

}  // namespace

void ValidateLegacyStackConfig(const LegacyStackConfig& config) {
  RequireOrderedConfig(config.actuation_limits.thrust_min_n,
                       config.actuation_limits.thrust_max_n,
                       "actuation_limits.thrust_n");
  RequireOrderedConfig(config.rotor.speed_squared_min_rad2_s2,
                       config.rotor.speed_squared_max_rad2_s2,
                       "rotor.speed_squared_*");
  const Vector3& inertia = config.vehicle.inertia_diag_kg_m2;
  if ((inertia.array() <= 0.0).any()) {
    throw ConfigError("vehicle.inertia_kg_m2 components must be > 0");
  }
  // Principal moments of any rigid body satisfy the triangle inequality.
  // A relative slack of kInertiaTriangleSlack admits the flat-plate limit
  // (e.g. Izz == Ixx + Iyy) despite decimal rounding in the YAML.
  constexpr double kInertiaTriangleSlack = 1e-9;
  const double slack = kInertiaTriangleSlack * inertia.sum();
  if (inertia.x() + inertia.y() + slack < inertia.z() ||
      inertia.y() + inertia.z() + slack < inertia.x() ||
      inertia.z() + inertia.x() + slack < inertia.y()) {
    throw ConfigError(
        "vehicle.inertia_kg_m2 violates the triangle inequality "
        "(e.g. xx + yy >= zz); no rigid body has these principal moments");
  }
  if (config.pilot.hover_thrust_n < config.actuation_limits.thrust_min_n ||
      config.pilot.hover_thrust_n > config.actuation_limits.thrust_max_n) {
    throw ConfigError(
        "pilot.hover_thrust_n must lie within actuation_limits.thrust_n");
  }
  if (config.rotor.spin_up_reaction_torque &&
      !(config.rotor.time_constant_s > 0.0)) {
    // With an instantaneous rotor response the rotor acceleration, and hence
    // the reaction torque, is undefined (it would depend on the physics
    // step). No real motor is instantaneous.
    throw ConfigError(
        "rotor.spin_up_reaction_torque requires rotor.time_constant_s > 0");
  }
  if (config.flight_controller.type == FlightControllerType::kMagisV2 &&
      config.rotor.layout != RotorLayout::kMagisQuadX) {
    // The firmware's QUADX mixer outputs M0..M3 in this layout.
    throw ConfigError(
        "flight_controller.type magisv2 requires rotor.layout magis_quad_x");
  }
  if (!(config.controller.period_s > 0.0) ||
      config.controller.period_s > 1.0) {
    throw ConfigError("controller.period_s must be in (0, 1] s");
  }
}

LegacyStackConfig LoadLegacyStackConfigFile(
    const std::string& path,
    const std::optional<std::string>& flight_controller_type) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw ConfigError("cannot read configuration file '" + path +
                      "': " + error.what());
  }
  if (flight_controller_type) {
    if (!root.IsMap() || !root["flight_controller"] ||
        !root["flight_controller"].IsMap()) {
      throw ConfigError("missing required parameter 'flight_controller'");
    }
    root["flight_controller"]["type"] = *flight_controller_type;
  }
  return ParseRoot(root);
}

LegacyStackConfig LoadLegacyStackConfigString(const std::string& yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& error) {
    throw ConfigError(std::string("cannot parse configuration: ") +
                      error.what());
  }
  return ParseRoot(root);
}

std::string DescribeLegacyStackConfig(const LegacyStackConfig& config) {
  std::ostringstream out;
  out << std::setprecision(6);
  const LegacyVehicleParams& v = config.vehicle;
  out << "vehicle: mass=" << v.mass_kg << " kg, g=" << v.gravity_m_s2
      << " m/s^2, inertia(xx,yy,zz)=(" << v.inertia_diag_kg_m2.transpose()
      << ") kg m^2, rotor_inertia=" << v.rotor_inertia_kg_m2
      << " kg m^2, arm=" << v.arm_length_m << " m, drag(N,E,D)=("
      << v.linear_drag_ned_n_s_m.transpose() << ") N s/m ("
      << (v.drag_model == DragModel::kRotorAndBody
              ? "unused; drag model rotor_and_body"
              : "drag model legacy_linear")
      << "), "
      << "legacy_double_arm_torque="
      << (v.legacy_double_arm_torque ? "true" : "false") << "\n";
  const LegacyRotorParams& r = config.rotor;
  out << "rotor: layout=" << RotorLayoutName(r.layout)
      << ", kt=" << r.thrust_coefficient_n_s2
      << " N s^2, kd=" << r.yaw_drag_coefficient_n_m_s2
      << " N m s^2, w^2 in [" << r.speed_squared_min_rad2_s2 << ", "
      << r.speed_squared_max_rad2_s2 << "] rad^2/s^2\n";
  const LegacyActuationLimits& a = config.actuation_limits;
  out << "actuation: thrust in [" << a.thrust_min_n << ", " << a.thrust_max_n
      << "] N, |torque|(roll,pitch,yaw) <= ("
      << a.torque_limit_n_m.transpose() << ") N m\n";
  out << "legacy controller (used only with flight_controller: legacy): period="
      << config.controller.period_s
      << " s (cascade position -> attitude -> rate -> mixer), "
      << "position error_frame="
      << PositionErrorFrameName(config.controller.position.error_frame)
      << "\n";
  out << "rotor time constant=" << r.time_constant_s << " s; battery "
      << (config.battery.enabled ? "enabled" : "disabled");
  if (config.battery.enabled) {
    out << " (" << config.battery.capacity_ah * 1000.0 << " mAh, R="
        << config.battery.internal_resistance_ohm << " ohm, V_ref="
        << config.battery.reference_voltage_v << " V)";
  }
  out << "; command latency=" << config.latency.command_s << " s\n";
  const StateErrorParams& e = config.state_error;
  out << "state error " << (e.enabled ? "enabled" : "disabled");
  if (e.enabled) {
    out << " (seed " << e.seed << ", attitude bias std "
        << e.attitude_bias_std_rad << " rad, gyro noise std "
        << e.gyro_noise_std_rad_s << " rad/s)";
  }
  const WindParams& w = config.wind;
  out << "; wind " << (w.enabled ? "enabled" : "disabled");
  if (w.enabled) {
    out << " (seed " << w.seed << ", mean ENU (" << w.mean_enu_m_s.transpose()
        << ") m/s, gust std " << w.gust_std_m_s << " m/s, tau "
        << w.gust_time_constant_s << " s)";
  }
  out << "\n";
  out << "flight_controller: "
      << FlightControllerTypeName(config.flight_controller.type);
  if (config.flight_controller.type == FlightControllerType::kMagisV2) {
    const MagisV2Params& m = config.flight_controller.magisv2;
    out << " (busy loop " << m.busy_loop_period_us
        << " us, IMU seed " << m.imu.seed << ", baro ground altitude "
        << m.baro.ground_altitude_msl_m << " m MSL, field NED ("
        << m.magnetic_field_ned_ut.transpose() << ") uT)";
  }
  out << "\n";
  out << "pilot: initial_mode=" << FlightModeName(config.pilot.initial_mode)
      << ", hover_thrust=" << config.pilot.hover_thrust_n
      << " N, position_hold_target_enu=("
      << config.pilot.position_hold_target_enu_m.transpose() << ") m";
  return out.str();
}

}  // namespace pluto_x
