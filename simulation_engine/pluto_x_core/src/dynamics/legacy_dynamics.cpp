// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/legacy_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/dynamics/aerodynamic_drag.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

void RequireFiniteInputs(const VehicleStateNed& state,
                         const BodyWrenchCommand& achieved,
                         double net_rotor_speed_rad_s, const Vector3& wind) {
  if (!IsFinite(state) || !IsFinite(achieved) ||
      !IsFinite(net_rotor_speed_rad_s) || !IsFinite(wind)) {
    throw std::invalid_argument("legacy dynamics inputs must be finite");
  }
}

}  // namespace

double RollPitchTorqueScale(const LegacyVehicleParams& vehicle) {
  return vehicle.legacy_double_arm_torque ? vehicle.arm_length_m : 1.0;
}

Vector3 GyroscopicTorque(const Vector3& body_rate_frd_rad_s,
                         double net_rotor_speed_rad_s,
                         const LegacyVehicleParams& vehicle) {
  const double jp = vehicle.rotor_inertia_kg_m2;
  const double o = net_rotor_speed_rad_s;
  const double p = body_rate_frd_rad_s.x();
  const double q = body_rate_frd_rad_s.y();
  if (vehicle.legacy_gyroscopic_form) {
    return Vector3(-jp * p * o, jp * q * o, 0.0);
  }
  // Rotor angular momentum: CCW-from-above rotors (s = +1) spin about -z_FRD.
  const Vector3 rotor_momentum_frd(0.0, 0.0, -jp * o);
  return -body_rate_frd_rad_s.cross(rotor_momentum_frd);
}

EulerAnglesZyx EulerRatesFromBodyRates(const EulerAnglesZyx& attitude,
                                       const Vector3& body_rate_frd_rad_s) {
  const double p = body_rate_frd_rad_s.x();
  const double q = body_rate_frd_rad_s.y();
  const double r = body_rate_frd_rad_s.z();
  const double sin_roll = std::sin(attitude.roll_rad);
  const double cos_roll = std::cos(attitude.roll_rad);
  const double cos_pitch = std::cos(attitude.pitch_rad);
  const double tan_pitch = std::tan(attitude.pitch_rad);

  EulerAnglesZyx rates;
  rates.roll_rad = p + sin_roll * tan_pitch * q + cos_roll * tan_pitch * r;
  rates.pitch_rad = cos_roll * q - sin_roll * r;
  rates.yaw_rad = (sin_roll / cos_pitch) * q + (cos_roll / cos_pitch) * r;
  return rates;
}

ExternalWrenchNedFrd ComputeLegacyExternalWrench(
    const VehicleStateNed& state, const BodyWrenchCommand& achieved,
    double net_rotor_speed_rad_s, const LegacyVehicleParams& vehicle,
    const Vector3& wind_velocity_ned_m_s, double rotor_speed_sum_rad_s) {
  RequireFiniteInputs(state, achieved, net_rotor_speed_rad_s,
                      wind_velocity_ned_m_s);

  const Matrix3 rotation_ned_frd = frames::RotationFromEulerZyx(state.attitude);
  const Vector3 thrust_ned_n =
      rotation_ned_frd * Vector3(0.0, 0.0, -achieved.thrust_n);
  const Vector3 drag_ned_n = ComputeDragForceNed(
      state, wind_velocity_ned_m_s, rotor_speed_sum_rad_s, vehicle);

  const double l = RollPitchTorqueScale(vehicle);
  const Vector3& u = achieved.torque_frd_n_m;

  ExternalWrenchNedFrd wrench;
  wrench.force_ned_n = thrust_ned_n + drag_ned_n;
  wrench.torque_frd_n_m =
      Vector3(l * u.x(), l * u.y(), u.z()) +
      GyroscopicTorque(state.body_rate_frd_rad_s, net_rotor_speed_rad_s,
                       vehicle);
  return wrench;
}

LegacyStateDerivatives ComputeLegacyDerivatives(
    const VehicleStateNed& state, const BodyWrenchCommand& achieved,
    double net_rotor_speed_rad_s, const LegacyVehicleParams& vehicle,
    const Vector3& wind_velocity_ned_m_s, double rotor_speed_sum_rad_s) {
  RequireFiniteInputs(state, achieved, net_rotor_speed_rad_s,
                      wind_velocity_ned_m_s);

  // Written term by term from kwad.cpp quad_dynamics() so that a reviewer
  // can compare line by line. (ComputeLegacyExternalWrench is the
  // independent rigid-body formulation; the tests check they agree.)
  const double phi = state.attitude.roll_rad;
  const double theta = state.attitude.pitch_rad;
  const double psi = state.attitude.yaw_rad;
  const double u1 = achieved.thrust_n;
  const double u2 = achieved.torque_frd_n_m.x();
  const double u3 = achieved.torque_frd_n_m.y();
  const double u4 = achieved.torque_frd_n_m.z();
  const double m = vehicle.mass_kg;
  const double g = vehicle.gravity_m_s2;
  const double l = RollPitchTorqueScale(vehicle);
  const double jx = vehicle.inertia_diag_kg_m2.x();
  const double jy = vehicle.inertia_diag_kg_m2.y();
  const double jz = vehicle.inertia_diag_kg_m2.z();
  // kwad.cpp: -kdx * x_dot etc.; generalised to ComputeDragForceNed().
  const Vector3 drag = ComputeDragForceNed(state, wind_velocity_ned_m_s,
                                           rotor_speed_sum_rad_s, vehicle);
  const double p = state.body_rate_frd_rad_s.x();
  const double q = state.body_rate_frd_rad_s.y();
  const double r = state.body_rate_frd_rad_s.z();

  using std::cos;
  using std::sin;
  LegacyStateDerivatives d;
  d.acceleration_ned_m_s2.x() =
      (-(cos(phi) * sin(theta) * cos(psi) + sin(psi) * sin(phi)) * u1 +
       drag.x()) / m;
  d.acceleration_ned_m_s2.y() =
      (-(cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)) * u1 +
       drag.y()) / m;
  d.acceleration_ned_m_s2.z() =
      ((-(cos(phi) * cos(theta)) * u1 + drag.z()) / m) + g;

  // gyro = (-jp p o, +jp q o, 0) in kwad.cpp; see GyroscopicTorque().
  const Vector3 gyro =
      GyroscopicTorque(state.body_rate_frd_rad_s, net_rotor_speed_rad_s,
                       vehicle);
  d.body_angular_accel_frd_rad_s2.x() =
      (q * r * (jy - jz) + gyro.x() + l * u2) / jx;
  d.body_angular_accel_frd_rad_s2.y() =
      (p * r * (jz - jx) + gyro.y() + l * u3) / jy;
  d.body_angular_accel_frd_rad_s2.z() =
      (p * q * (jx - jy) + gyro.z() + u4) / jz;

  d.euler_rates_rad_s =
      EulerRatesFromBodyRates(state.attitude, state.body_rate_frd_rad_s);
  return d;
}

LegacyReferenceIntegrator::LegacyReferenceIntegrator(
    const LegacyVehicleParams& vehicle, const LegacyStateClampParams& clamps)
    : vehicle_(vehicle), clamps_(clamps) {}

LegacyStateDerivatives LegacyReferenceIntegrator::Step(
    VehicleStateNed& state, const BodyWrenchCommand& achieved,
    double net_rotor_speed_rad_s, double timestep_s,
    const Vector3& wind_velocity_ned_m_s, double rotor_speed_sum_rad_s) const {
  if (!(timestep_s > 0.0) || !IsFinite(timestep_s)) {
    throw std::invalid_argument("integrator timestep must be > 0");
  }
  const LegacyStateDerivatives d =
      ComputeLegacyDerivatives(state, achieved, net_rotor_speed_rad_s,
                               vehicle_, wind_velocity_ned_m_s,
                               rotor_speed_sum_rad_s);

  state.velocity_ned_m_s += d.acceleration_ned_m_s2 * timestep_s;
  state.position_ned_m += state.velocity_ned_m_s * timestep_s;
  if (state.position_ned_m.z() > 0.0) {  // below ground (NED z down)
    state.position_ned_m.z() = 0.0;
    state.velocity_ned_m_s.z() = std::min(state.velocity_ned_m_s.z(), 0.0);
  }

  const Vector3& rate_limit = clamps_.body_rate_limit_rad_s;
  Vector3& rate = state.body_rate_frd_rad_s;
  rate += d.body_angular_accel_frd_rad_s2 * timestep_s;
  for (int axis = 0; axis < 3; ++axis) {
    rate[axis] = ClampSymmetric(rate[axis], rate_limit[axis]);
  }

  const EulerAnglesZyx& limit = clamps_.attitude_limit_rad;
  EulerAnglesZyx& att = state.attitude;
  att.roll_rad = ClampSymmetric(
      att.roll_rad + d.euler_rates_rad_s.roll_rad * timestep_s,
      limit.roll_rad);
  att.pitch_rad = ClampSymmetric(
      att.pitch_rad + d.euler_rates_rad_s.pitch_rad * timestep_s,
      limit.pitch_rad);
  att.yaw_rad = ClampSymmetric(
      att.yaw_rad + d.euler_rates_rad_s.yaw_rad * timestep_s, limit.yaw_rad);
  return d;
}

}  // namespace pluto_x
