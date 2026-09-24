// Copyright 2026 AVATIC contributors.
//
// The kwad.cpp quad_dynamics() model, split into three explicit pieces.
//
// 1. ComputeLegacyDerivatives()  the legacy equations of motion, verbatim:
//
//    a_ned   = ( -u1 * R_ned_frd * e_z  -  diag(kdx, kdy, kdz) v_ned ) / m
//              + g e_z
//    p_dot   = ( q r (Jy - Jz) - jp p o + l u2 ) / Jx
//    q_dot   = ( p r (Jz - Jx) + jp q o + l u3 ) / Jy
//    r_dot   = ( p q (Jx - Jy)           + u4 ) / Jz
//    Euler rates from the standard Z-Y-X kinematics.
//
//    o = w1 - w2 + w3 - w4 is the legacy net rotor speed.
//
// 2. ComputeLegacyExternalWrench()  the same model rewritten as an external
//    force + torque acting on a rigid body. The rigid-body (Newton-Euler)
//    equations
//
//        m a   = F_ext + m g e_z
//        J w_dot = tau_ext - w x (J w)
//
//    reproduce the legacy equations exactly when (for diagonal J)
//
//        F_ext   = -u1 R e_z - diag(kd) v
//        tau_ext = ( l u2 - jp p o,  l u3 + jp q o,  u4 )
//
//    because -(w x J w) = ( q r (Jy - Jz), p r (Jz - Jx), p q (Jx - Jy) ).
//    This is what the Gazebo plugin applies; Gazebo's physics engine then
//    integrates the rigid body (gravity supplied by the Gazebo world).
//    The equivalence is verified in test/test_legacy_dynamics.cpp.
//
// 3. LegacyReferenceIntegrator  the kwad.cpp explicit-Euler integrator with
//    its state clamps. Used ONLY for offline reference runs and tests, never
//    as the Gazebo vehicle dynamics.
//
// In the equations above, "l u2" / "l u3" are written with
// l = RollPitchTorqueScale(vehicle): the arm length when
// vehicle.legacy_double_arm_torque is set (kwad.cpp behaviour), otherwise 1.
//
// ASSUMPTIONS / INHERITED QUIRKS (not validated, see docs/legacy_port.md):
//  * With legacy_double_arm_torque, u2, u3 already contain the arm length
//    (mixer: u2 = kt l dw^2) and the dynamics multiply by l again, so the
//    effective roll/pitch torque is kt l^2 dw^2.
//  * With legacy_gyroscopic_form, the gyroscopic terms are (-jp p o) on roll
//    and (+jp q o) on pitch (kwad.cpp); otherwise the physical
//    -w x h form (see LegacyVehicleParams). "-jp p o" / "+jp q o" in the
//    equations above stand for GyroscopicTorque().
//  * Drag: "- diag(kd) v" above stands for ComputeDragForceNed()
//    (aerodynamic_drag.hpp): kwad.cpp's linear world-frame drag, or rotor +
//    body drag; both act on the air-relative velocity v - wind.

#ifndef PLUTO_X_DYNAMICS_LEGACY_DYNAMICS_HPP_
#define PLUTO_X_DYNAMICS_LEGACY_DYNAMICS_HPP_

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

struct LegacyStateDerivatives {
  Vector3 acceleration_ned_m_s2{Vector3::Zero()};
  Vector3 body_angular_accel_frd_rad_s2{Vector3::Zero()};
  EulerAnglesZyx euler_rates_rad_s;
};

/// Factor applied to the roll/pitch torque command by the dynamics:
/// arm_length_m if vehicle.legacy_double_arm_torque, else 1.
double RollPitchTorqueScale(const LegacyVehicleParams& vehicle);

/// Rotor gyroscopic torque (FRD) for vehicle.legacy_gyroscopic_form, see
/// LegacyVehicleParams.
Vector3 GyroscopicTorque(const Vector3& body_rate_frd_rad_s,
                         double net_rotor_speed_rad_s,
                         const LegacyVehicleParams& vehicle);

/// Legacy equations of motion. Drag acts on the air-relative velocity
/// v - wind. Precondition: finite inputs (checked).
LegacyStateDerivatives ComputeLegacyDerivatives(
    const VehicleStateNed& state, const BodyWrenchCommand& achieved,
    double net_rotor_speed_rad_s, const LegacyVehicleParams& vehicle,
    const Vector3& wind_velocity_ned_m_s = Vector3::Zero(),
    double rotor_speed_sum_rad_s = 0.0);

/// Z-Y-X Euler angle rates from body rates. Singular at pitch = +/-pi/2.
EulerAnglesZyx EulerRatesFromBodyRates(const EulerAnglesZyx& attitude,
                                       const Vector3& body_rate_frd_rad_s);

struct ExternalWrenchNedFrd {
  /// Total non-gravitational force, NED world frame.
  Vector3 force_ned_n{Vector3::Zero()};
  /// Total external torque about the centre of mass, FRD body frame.
  Vector3 torque_frd_n_m{Vector3::Zero()};
};

/// External wrench equivalent to the legacy model (see header comment).
/// Gravity is NOT included. Precondition: finite inputs (checked).
ExternalWrenchNedFrd ComputeLegacyExternalWrench(
    const VehicleStateNed& state, const BodyWrenchCommand& achieved,
    double net_rotor_speed_rad_s, const LegacyVehicleParams& vehicle,
    const Vector3& wind_velocity_ned_m_s = Vector3::Zero(),
    double rotor_speed_sum_rad_s = 0.0);

/// kwad.cpp integrator, reproduced step for step:
///   derivatives from the current state, v += a dt, x += v_new dt,
///   w += w_dot dt (clamped), euler += euler_rate_old dt (clamped).
///
/// Fixed relative to kwad.cpp: the ground clamp. kwad.cpp clamped
/// `z < 0 -> 0` in NED, which forbade flying ABOVE the ground and let the
/// vertical velocity grow unbounded. Here the ground is z_ned = 0: the
/// vehicle cannot go below it and downward velocity is zeroed on contact.
class LegacyReferenceIntegrator {
 public:
  LegacyReferenceIntegrator(const LegacyVehicleParams& vehicle,
                            const LegacyStateClampParams& clamps);

  /// Advances `state` by timestep_s and returns the derivatives used.
  /// Preconditions (checked, throw std::invalid_argument): finite inputs,
  /// timestep_s > 0.
  LegacyStateDerivatives Step(
      VehicleStateNed& state, const BodyWrenchCommand& achieved,
      double net_rotor_speed_rad_s, double timestep_s,
      const Vector3& wind_velocity_ned_m_s = Vector3::Zero(),
      double rotor_speed_sum_rad_s = 0.0) const;

 private:
  LegacyVehicleParams vehicle_;
  LegacyStateClampParams clamps_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_LEGACY_DYNAMICS_HPP_
