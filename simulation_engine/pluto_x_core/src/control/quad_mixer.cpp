// Copyright 2026 AVATIC contributors.

#include "pluto_x/control/quad_mixer.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

/// |det| of the allocation matrix below which it is treated as singular,
/// relative to the product of its row scales.
constexpr double kSingularityTolerance = 1e-9;

/// A rotor counts as saturated when clamping moved its w^2 by more than this
/// fraction of the upper limit (ignores round-off from the matrix inverse).
constexpr double kSaturationRelativeTolerance = 1e-9;

}  // namespace

std::array<RotorGeometry, kRotorCount> MakeRotorGeometry(RotorLayout layout,
                                                         double arm_length_m) {
  const double d = arm_length_m;
  switch (layout) {
    case RotorLayout::kLegacyPlus:
      return {{{d, 0.0, +1.0},     // 1 front
               {0.0, d, -1.0},     // 2 right
               {-d, 0.0, +1.0},    // 3 rear
               {0.0, -d, -1.0}}};  // 4 left
    case RotorLayout::kMagisQuadX: {
      const double c = d / std::sqrt(2.0);
      return {{{-c, c, -1.0},     // M0 rear-right, CW
               {c, c, +1.0},      // M1 front-right, CCW
               {-c, -c, +1.0},    // M2 rear-left, CCW
               {c, -c, -1.0}}};   // M3 front-left, CW
    }
  }
  throw std::invalid_argument("unknown rotor layout");
}

QuadMixer::QuadMixer(const LegacyRotorParams& rotor, double arm_length_m)
    : rotor_(rotor) {
  if (!(rotor.thrust_coefficient_n_s2 > 0.0) ||
      !(rotor.yaw_drag_coefficient_n_m_s2 > 0.0) || !(arm_length_m > 0.0)) {
    throw std::invalid_argument(
        "QuadMixer requires kt > 0, kd > 0 and arm length > 0");
  }
  RequireOrdered(rotor.speed_squared_min_rad2_s2,
                 rotor.speed_squared_max_rad2_s2, "rotor speed^2 limits");
  if (rotor.speed_squared_min_rad2_s2 < 0.0) {
    throw std::invalid_argument("rotor speed^2 lower limit must be >= 0");
  }

  geometry_ = MakeRotorGeometry(rotor.layout, arm_length_m);
  const double kt = rotor.thrust_coefficient_n_s2;
  const double kd = rotor.yaw_drag_coefficient_n_m_s2;
  for (int i = 0; i < kRotorCount; ++i) {
    const RotorGeometry& g = geometry_[i];
    allocation_(0, i) = kt;
    allocation_(1, i) = -kt * g.y_frd_m;
    allocation_(2, i) = kt * g.x_frd_m;
    allocation_(3, i) = kd * g.yaw_reaction_sign;
  }
  const double scale = kt * (kt * arm_length_m) * (kt * arm_length_m) * kd;
  if (std::abs(allocation_.determinant()) < kSingularityTolerance * scale) {
    throw std::invalid_argument("rotor allocation matrix is singular");
  }
  allocation_inverse_ = allocation_.inverse();
}

MixerOutput QuadMixer::Mix(const BodyWrenchCommand& command) const {
  if (!IsFinite(command)) {
    throw std::invalid_argument("mixer command must be finite");
  }
  const Vector4 wrench(command.thrust_n, command.torque_frd_n_m.x(),
                       command.torque_frd_n_m.y(), command.torque_frd_n_m.z());
  const Vector4 requested = allocation_inverse_ * wrench;

  std::array<double, kRotorCount> speeds{};
  bool saturated = false;
  for (int i = 0; i < kRotorCount; ++i) {
    const double clamped =
        Clamp(requested[i], rotor_.speed_squared_min_rad2_s2,
              rotor_.speed_squared_max_rad2_s2);
    saturated = saturated ||
                std::abs(clamped - requested[i]) >
                    kSaturationRelativeTolerance *
                        rotor_.speed_squared_max_rad2_s2;
    speeds[i] = std::sqrt(clamped);
  }
  MixerOutput output = FromRotorSpeeds(speeds);
  output.saturated = saturated;
  return output;
}

MixerOutput QuadMixer::FromRotorSpeeds(
    const std::array<double, kRotorCount>& rotor_speed_rad_s) const {
  MixerOutput output;
  Vector4 speed_squared;
  for (int i = 0; i < kRotorCount; ++i) {
    const double w = rotor_speed_rad_s[i];
    if (!IsFinite(w) || w < 0.0) {
      throw std::invalid_argument("rotor speeds must be finite and >= 0");
    }
    speed_squared[i] = w * w;
    output.rotor_speed_rad_s[i] = w;
    output.rotor_speed_squared_rad2_s2[i] = w * w;
    output.net_rotor_speed_rad_s += geometry_[i].yaw_reaction_sign * w;
    output.rotor_speed_sum_rad_s += w;
  }
  const Vector4 wrench = allocation_ * speed_squared;
  output.achieved.thrust_n = wrench[0];
  output.achieved.torque_frd_n_m = Vector3(wrench[1], wrench[2], wrench[3]);
  return output;
}

}  // namespace pluto_x
