// Copyright 2026 AVATIC contributors.

#include "pluto_x/dynamics/aerodynamic_drag.hpp"

#include <stdexcept>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/common/validation.hpp"

namespace pluto_x {

Vector3 ComputeDragForceNed(const VehicleStateNed& state,
                            const Vector3& wind_velocity_ned_m_s,
                            double rotor_speed_sum_rad_s,
                            const LegacyVehicleParams& vehicle) {
  if (!IsFinite(state) || !IsFinite(wind_velocity_ned_m_s) ||
      !IsFinite(rotor_speed_sum_rad_s) || rotor_speed_sum_rad_s < 0.0) {
    throw std::invalid_argument("drag inputs must be finite, rotor sum >= 0");
  }
  const Vector3 air_velocity_ned =
      state.velocity_ned_m_s - wind_velocity_ned_m_s;

  switch (vehicle.drag_model) {
    case DragModel::kLegacyLinear:
      return -vehicle.linear_drag_ned_n_s_m.cwiseProduct(air_velocity_ned);

    case DragModel::kRotorAndBody: {
      const Matrix3 rotation_ned_frd =
          frames::RotationFromEulerZyx(state.attitude);
      const Vector3 v_body = rotation_ned_frd.transpose() * air_velocity_ned;
      const Vector3 rotor_drag_body =
          -vehicle.rotor_drag_coefficient_n_s_m_rad * rotor_speed_sum_rad_s *
          Vector3(v_body.x(), v_body.y(), 0.0);
      const Vector3 body_drag_body =
          -0.5 * vehicle.air_density_kg_m3 *
          vehicle.body_drag_area_frd_m2.cwiseProduct(
              v_body.cwiseAbs().cwiseProduct(v_body));
      return rotation_ned_frd * (rotor_drag_body + body_drag_body);
    }
  }
  throw std::invalid_argument("unknown drag model");
}

}  // namespace pluto_x
