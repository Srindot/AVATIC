// Copyright 2026 AVATIC contributors.
//
// Barometric pressure sensor.
//
// ASSUMPTIONS:
//  * pressure at the ground (simulator origin) from the ISA troposphere at
//    ground_altitude_msl_m: p_g = p0 (1 - L h_g / T0)^(g M / (R L));
//  * above the ground, a lapse-rate atmosphere whose air temperature AT THE
//    GROUND is temperature_c: p(h) = p_g (1 - L h / T_g)^(g M / (R L)),
//    h = height above the origin. The pressure-height gradient therefore
//    matches the air temperature the sensor reports (the MagisV2 altitude
//    formula uses that temperature; with the ISA temperature instead, a
//    30 degC day gives a ~6 % altitude scale error);
//  * white noise; constant sensor temperature.
// Values are in docs/pluto_x_parameters.md.

#ifndef PLUTO_X_SENSORS_BARO_MODEL_HPP_
#define PLUTO_X_SENSORS_BARO_MODEL_HPP_

#include <cstdint>
#include <random>

namespace pluto_x {

struct BaroParams {
  std::uint64_t seed{4};
  double sea_level_pressure_pa{101325.0};
  double ground_altitude_msl_m{0.0};
  double temperature_c{25.0};
  double noise_std_pa{0.0};
};

struct BaroSample {
  double pressure_pa{0.0};
  double temperature_c{0.0};
};

/// ISA pressure at an altitude above mean sea level (m), for p0 (Pa).
/// Valid below 11 km.
double IsaPressurePa(double altitude_msl_m, double sea_level_pressure_pa);

class BaroModel {
 public:
  /// Throws std::invalid_argument for invalid parameters.
  explicit BaroModel(const BaroParams& params);

  /// height_above_origin_m: ENU z of the vehicle (checked finite).
  BaroSample Sample(double height_above_origin_m);

  /// Noise-free pressure at a height above the origin (checked finite).
  double PressureAtHeightPa(double height_above_origin_m) const;

 private:
  BaroParams params_;
  std::mt19937_64 engine_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

}  // namespace pluto_x

#endif  // PLUTO_X_SENSORS_BARO_MODEL_HPP_
