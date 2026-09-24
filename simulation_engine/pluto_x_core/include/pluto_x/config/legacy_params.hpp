// Copyright 2026 AVATIC contributors.
//
// Parameter structures for the legacy (kwad.cpp-derived) vehicle model and
// controller stack.
//
// PROVENANCE: every default value shipped in config/legacy_kwad.yaml is copied
// from the legacy PlutoX-ROS-Joystick-Control src/kwad.cpp #defines. These
// values describe the vehicle that kwad.cpp simulated (1.4 kg, 0.56 m arm).
// They are NOT Pluto X parameters and have never been validated against the
// Pluto X hardware. See docs/legacy_port.md.
//
// All quantities are SI. Suffixes give units: _kg, _m, _s, _n (newton),
// _n_m (newton metre), _rad, _rad_s, _rad2_s2 (rad^2/s^2).

#ifndef PLUTO_X_CONFIG_LEGACY_PARAMS_HPP_
#define PLUTO_X_CONFIG_LEGACY_PARAMS_HPP_

#include <cstdint>

#include "pluto_x/common/math_types.hpp"
#include "pluto_x/dynamics/battery_model.hpp"
#include "pluto_x/sensors/baro_model.hpp"
#include "pluto_x/sensors/imu_model.hpp"

namespace pluto_x {

/// Gains of one legacy PID loop.
///
/// Legacy semantics (kwad.cpp):
///  * The integral only accumulates while |error| < integral_window.
///  * The *integral term* (ki * integral) is clamped, the accumulator is not.
///  * The derivative term is kd * (a measured derivative signal supplied by
///    the caller, e.g. velocity or angular acceleration), NOT kd * d(error)/dt.
///    Several legacy kd values are negative for this reason.
struct LegacyPidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  /// |error| below which the integrator accumulates. Same unit as the error.
  double integral_window{0.0};
};

/// Aerodynamic drag model (see pluto_x/dynamics/aerodynamic_drag.hpp).
enum class DragModel {
  kLegacyLinear,   ///< kwad.cpp: -c * v in world axes
  kRotorAndBody,   ///< rotor drag + quadratic body drag
};

/// Rigid-body and aerodynamic parameters of the legacy dynamics model.
struct LegacyVehicleParams {
  double mass_kg{0.0};
  double gravity_m_s2{0.0};
  /// Principal moments of inertia about body x, y, z. The inertia tensor is
  /// assumed diagonal (ASSUMPTION inherited from kwad.cpp).
  Vector3 inertia_diag_kg_m2{Vector3::Zero()};
  /// Rotor inertia "jp" used in the legacy gyroscopic term.
  double rotor_inertia_kg_m2{0.0};
  /// Distance from the centre of mass to each rotor axis (legacy "l").
  double arm_length_m{0.0};
  /// kwad.cpp quirk: the dynamics multiply the roll/pitch torque command by
  /// arm_length_m although the mixer already included it, giving an
  /// effective torque of kt l^2 dw^2. true reproduces kwad.cpp; false applies
  /// the torque command as-is (physically consistent).
  bool legacy_double_arm_torque{false};
  /// kwad.cpp gyroscopic term (-jp p o, +jp q o, 0) if true; otherwise the
  /// physical rotor-momentum term -w x h with h = -jp o e_z (FRD), i.e.
  /// (jp o q, -jp o p, 0).
  bool legacy_gyroscopic_form{false};
  DragModel drag_model{DragModel::kLegacyLinear};
  /// kLegacyLinear: coefficients along world N, E, D: F = -diag(c) v.
  Vector3 linear_drag_ned_n_s_m{Vector3::Zero()};
  /// kRotorAndBody: rotor drag per (m/s of in-plane air speed) per (rad/s of
  /// summed rotor speed).
  double rotor_drag_coefficient_n_s_m_rad{0.0};
  /// kRotorAndBody: drag coefficient x reference area per FRD body axis.
  Vector3 body_drag_area_frd_m2{Vector3::Zero()};
  double air_density_kg_m3{0.0};
};

/// Rotor arrangement (see QuadMixer for the exact geometry and order).
enum class RotorLayout {
  /// kwad.cpp "+" frame: rotors 1..4 = front, right, rear, left.
  kLegacyPlus,
  /// MagisV2 MIXER_QUADX: rotors M0..M3 = rear-right, front-right,
  /// rear-left, front-left; M1 and M2 spin CCW (seen from above), as the
  /// production firmware's yaw control requires (see QuadMixer).
  kMagisQuadX,
};

/// Rotor model: thrust = kt * w^2, yaw reaction torque = kd * w^2.
struct LegacyRotorParams {
  RotorLayout layout{RotorLayout::kLegacyPlus};
  /// First-order rotor speed time constant; 0 = instantaneous (legacy).
  double time_constant_s{0.0};
  /// Include the yaw reaction torque of accelerating rotors,
  /// sum(s_i * J_rotor * dw_i/dt) (kwad.cpp: no).
  bool spin_up_reaction_torque{false};
  double thrust_coefficient_n_s2{0.0};
  double yaw_drag_coefficient_n_m_s2{0.0};
  double speed_squared_min_rad2_s2{0.0};
  double speed_squared_max_rad2_s2{0.0};
};

/// Saturation limits on the collective thrust and body-torque commands
/// (legacy u1_min/max, u2..u4 limits).
struct LegacyActuationLimits {
  double thrust_min_n{0.0};
  double thrust_max_n{0.0};
  /// Symmetric limits for roll (u2), pitch (u3), yaw (u4) torque commands.
  Vector3 torque_limit_n_m{Vector3::Zero()};
};

/// Frame in which the position loop forms its position/velocity error.
enum class PositionErrorFrame {
  /// kwad.cpp: target rotated by yaw only, but position and velocity rotated
  /// by the FULL attitude. This couples tilt and altitude into the
  /// horizontal error (e.g. y_body contains sin(roll) * z). The coupling is
  /// stabilising for z_ned > 0, which is where kwad.cpp's inverted ground
  /// clamp kept the vehicle; above real ground it destabilises with
  /// altitude (reference integrator: stable at 1 m, divergent for targets
  /// >= 2 m). See docs/legacy_port.md, "Position-hold instability".
  kLegacyFullAttitude,
  /// Target, position and velocity all rotated by yaw only.
  kYawOnly,
};

struct LegacyPositionControllerParams {
  PositionErrorFrame error_frame{PositionErrorFrame::kYawOnly};
  LegacyPidGains north;  ///< legacy x loop
  LegacyPidGains east;   ///< legacy y loop
  LegacyPidGains down;   ///< legacy z loop
  double roll_limit_rad{0.0};   ///< legacy phi_max
  double pitch_limit_rad{0.0};  ///< legacy theta_max
};

struct LegacyAttitudeControllerParams {
  LegacyPidGains roll;
  LegacyPidGains pitch;
  LegacyPidGains yaw;
  /// Symmetric limits on the body-rate setpoint (legacy p_max, q_max, r_max).
  Vector3 body_rate_limit_rad_s{Vector3::Zero()};
};

struct LegacyRateControllerParams {
  LegacyPidGains roll;
  LegacyPidGains pitch;
  LegacyPidGains yaw;
};

struct LegacyControllerParams {
  /// Fixed controller period (legacy Ts).
  double period_s{0.0};
  LegacyPositionControllerParams position;
  LegacyAttitudeControllerParams attitude;
  LegacyRateControllerParams rate;
};

enum class FlightMode {
  /// Pilot commands collective thrust and roll/pitch angles; yaw held at 0.
  kManualAttitude,
  /// Position loop drives roll/pitch/thrust toward a fixed target.
  kPositionHold,
};

struct LegacyPilotParams {
  FlightMode initial_mode{FlightMode::kManualAttitude};
  /// true: the yaw setpoint is the heading at the first controller step
  /// (heading hold, as a flight controller does at arming). false: yaw
  /// setpoint fixed at 0 = north (kwad.cpp psi_des = 0).
  bool hold_initial_heading{true};
  /// Collective thrust commanded in manual_attitude mode.
  double hover_thrust_n{0.0};
  /// Position-hold target in the ENU world frame.
  Vector3 position_hold_target_enu_m{Vector3::Zero()};
};

/// State clamps applied by the *reference integrator only*. Gazebo physics
/// cannot (and should not) clamp the physical state.
struct LegacyStateClampParams {
  Vector3 body_rate_limit_rad_s{Vector3::Zero()};
  EulerAnglesZyx attitude_limit_rad;
};

/// Delay between a pilot command reaching the simulator and the controller
/// acting on it (models the Wi-Fi command link).
struct LatencyParams {
  double command_s{0.0};
};

/// Errors on the state the controller sees. A stand-in for a real sensor +
/// estimator chain until MagisV2 runs in the loop. Position and velocity
/// are passed through unchanged.
struct StateErrorParams {
  bool enabled{false};
  std::uint64_t seed{1};
  /// Constant roll/pitch estimate offset drawn once per run, N(0, std).
  double attitude_bias_std_rad{0.0};
  /// White noise on roll/pitch/yaw estimates, per controller step.
  double attitude_noise_std_rad{0.0};
  /// Constant gyro offset drawn once per run, N(0, std), per axis.
  double gyro_bias_std_rad_s{0.0};
  /// White noise on body rates, per controller step.
  double gyro_noise_std_rad_s{0.0};
};

/// Wind = mean + per-axis first-order Gauss-Markov gust.
struct WindParams {
  bool enabled{false};
  std::uint64_t seed{2};
  Vector3 mean_enu_m_s{Vector3::Zero()};
  /// Stationary standard deviation of the gust on each axis.
  double gust_std_m_s{0.0};
  double gust_time_constant_s{1.0};
};

/// Which flight controller flies the simulated vehicle.
enum class FlightControllerType {
  /// LegacyFlightController: kwad.cpp cascade on the (optionally corrupted)
  /// full state; pilot command fixed by `pilot`; ignores RC.
  kLegacy,
  /// pluto_x_magisv2::MagisFlightController: the production MagisV2
  /// firmware on simulated sensors, commanded by RC frames.
  kMagisV2,
};

/// Sensors and host options of the MagisV2 flight controller.
struct MagisV2Params {
  /// Simulated time between calls of the firmware's loop().
  std::uint32_t busy_loop_period_us{100};
  /// See pluto_x_magisv2::MagisHostOptions::preload_mag_calibration.
  bool preload_mag_calibration{true};
  /// sensor_from_body is FRD -> FLU (the firmware's frame), fixed.
  ImuParams imu;
  BaroParams baro;
  /// Earth magnetic field at the flying site, NED world frame.
  Vector3 magnetic_field_ned_ut{Vector3::Zero()};
  /// AK09916: 0.15 uT per LSB.
  double mag_counts_per_ut{0.0};
};

struct FlightControllerParams {
  FlightControllerType type{FlightControllerType::kLegacy};
  /// Read and validated only when type == kMagisV2.
  MagisV2Params magisv2;
};

/// Complete configuration of the legacy stack.
struct LegacyStackConfig {
  LegacyVehicleParams vehicle;
  LegacyRotorParams rotor;
  LegacyActuationLimits actuation_limits;
  LegacyControllerParams controller;
  LegacyPilotParams pilot;
  LegacyStateClampParams reference_state_clamps;
  BatteryParams battery;
  LatencyParams latency;
  StateErrorParams state_error;
  WindParams wind;
  FlightControllerParams flight_controller;
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONFIG_LEGACY_PARAMS_HPP_
