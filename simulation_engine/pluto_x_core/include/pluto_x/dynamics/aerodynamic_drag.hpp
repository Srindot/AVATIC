// Copyright 2026 AVATIC contributors.
//
// Aerodynamic drag force on the vehicle (NED world frame).
//
// Two models, selected by LegacyVehicleParams::drag_model:
//
// kLegacyLinear (kwad.cpp):
//   F = -diag(c_N, c_E, c_D) (v - w)            world axes
//
// kRotorAndBody:
//   v_b   = R^T (v - w)                         air-relative, FRD body
//   F_rot = -k_rd * S * (v_b,x, v_b,y, 0)       rotor drag (H-force / induced
//                                               drag), S = sum of rotor
//                                               speeds; zero with rotors
//                                               stopped
//   F_bod = -1/2 rho (CdA) o |v_b| o v_b        body parasitic drag, per body
//                                               axis (o = element-wise)
//   F     = R (F_rot + F_bod)
//
// ASSUMPTIONS (kRotorAndBody): rotor drag is linear in the in-plane
// air-relative velocity and in the summed rotor speed (the standard
// first-order blade-element result used e.g. by RotorS/PX4 SITL); no vertical
// rotor drag; body drag coefficients per body axis without cross-coupling.
// Parameter values: docs/pluto_x_parameters.md.

#ifndef PLUTO_X_DYNAMICS_AERODYNAMIC_DRAG_HPP_
#define PLUTO_X_DYNAMICS_AERODYNAMIC_DRAG_HPP_

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

/// Drag force in NED. Preconditions (checked, throw std::invalid_argument):
/// finite inputs, rotor_speed_sum_rad_s >= 0.
Vector3 ComputeDragForceNed(const VehicleStateNed& state,
                            const Vector3& wind_velocity_ned_m_s,
                            double rotor_speed_sum_rad_s,
                            const LegacyVehicleParams& vehicle);

}  // namespace pluto_x

#endif  // PLUTO_X_DYNAMICS_AERODYNAMIC_DRAG_HPP_
