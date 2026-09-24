// Copyright 2026 AVATIC contributors.
//
// Geometric quadrotor mixer (generalises kwad.cpp quad_motor_speed()).
//
// Each rotor i has a position (x_i, y_i) in the FRD body frame and a yaw
// reaction sign s_i (+1: reaction torque along +z_FRD, i.e. the propeller
// spins counter-clockwise seen from above). With thrust T_i = kt w_i^2
// acting along -z_FRD:
//
//   u1 (thrust)  = kt  * sum(w_i^2)
//   u2 (roll)    = kt  * sum(-y_i w_i^2)
//   u3 (pitch)   = kt  * sum( x_i w_i^2)
//   u4 (yaw)     = kd  * sum( s_i w_i^2)
//
// The 4x4 allocation matrix is inverted once at construction. Each w_i^2 is
// then clamped to its limits and the achieved wrench is recomputed from the
// clamped values.
//
// Layouts (d = arm_length):
//
//   kLegacyPlus (kwad.cpp, rotors 1..4):
//     1 front (+d, 0) s=+1   2 right (0, +d) s=-1
//     3 rear  (-d, 0) s=+1   4 left  (0, -d) s=-1
//     This reproduces the kwad.cpp equations exactly (tested).
//
//   kMagisQuadX (MagisV2 mixerQuadX, motors M0..M3), c = d / sqrt(2):
//     M0 rear-right (-c, +c) s=-1 (CW)    M1 front-right (+c, +c) s=+1 (CCW)
//     M2 rear-left  (-c, -c) s=+1 (CCW)   M3 front-left  (+c, -c) s=-1 (CW)
//     Spin directions are those the production firmware assumes: running
//     MagisV2 on the host (pluto_x_magisv2 test, yaw phase), a
//     counter-clockwise yaw rate is opposed by speeding up M1 and M2, which
//     only yields a clockwise reaction if M1/M2 spin counter-clockwise. This
//     is the reverse of the textbook Cleanflight QUADX layout. Still to be
//     confirmed by looking at a physical Pluto X.

#ifndef PLUTO_X_CONTROL_QUAD_MIXER_HPP_
#define PLUTO_X_CONTROL_QUAD_MIXER_HPP_

#include <array>

#include <Eigen/Core>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/dynamics/vehicle_state.hpp"

namespace pluto_x {

inline constexpr int kRotorCount = 4;

struct RotorGeometry {
  double x_frd_m{0.0};
  double y_frd_m{0.0};
  double yaw_reaction_sign{0.0};  ///< +1 or -1
};

/// Rotor geometry for a layout and arm length (see header comment).
std::array<RotorGeometry, kRotorCount> MakeRotorGeometry(RotorLayout layout,
                                                         double arm_length_m);

struct MixerOutput {
  /// Rotor order as defined by the layout.
  std::array<double, kRotorCount> rotor_speed_squared_rad2_s2{};
  std::array<double, kRotorCount> rotor_speed_rad_s{};
  /// Wrench produced by the clamped rotor speeds.
  BodyWrenchCommand achieved;
  /// Legacy "o" = sum(s_i w_i), used by the gyroscopic term.
  double net_rotor_speed_rad_s{0.0};
  /// sum(w_i), used by the rotor drag model.
  double rotor_speed_sum_rad_s{0.0};
  /// True if any rotor speed was clamped.
  bool saturated{false};
};

class QuadMixer {
 public:
  /// Throws std::invalid_argument for non-positive coefficients or arm
  /// length, unordered speed limits, or a singular allocation matrix.
  QuadMixer(const LegacyRotorParams& rotor, double arm_length_m);

  /// Precondition (checked, throws std::invalid_argument): command finite.
  MixerOutput Mix(const BodyWrenchCommand& command) const;

  /// Forward model: wrench and net rotor speed produced by the given rotor
  /// speeds (no clamping; `saturated` is false). Precondition (checked):
  /// speeds finite and >= 0.
  MixerOutput FromRotorSpeeds(
      const std::array<double, kRotorCount>& rotor_speed_rad_s) const;

  const std::array<RotorGeometry, kRotorCount>& geometry() const {
    return geometry_;
  }

 private:
  using Matrix4 = Eigen::Matrix4d;
  using Vector4 = Eigen::Vector4d;

  LegacyRotorParams rotor_;
  std::array<RotorGeometry, kRotorCount> geometry_;
  Matrix4 allocation_;          ///< w^2 -> (u1, u2, u3, u4)
  Matrix4 allocation_inverse_;  ///< (u1, u2, u3, u4) -> w^2
};

}  // namespace pluto_x

#endif  // PLUTO_X_CONTROL_QUAD_MIXER_HPP_
