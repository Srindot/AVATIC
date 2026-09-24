// Copyright 2026 AVATIC contributors.

#include "pluto_x/sensors/baro_model.hpp"

#include <cmath>
#include <stdexcept>

#include "pluto_x/common/validation.hpp"

namespace pluto_x {
namespace {

// ISA troposphere constants.
constexpr double kSeaLevelTemperatureK = 288.15;
constexpr double kLapseRateKPerM = 0.0065;
constexpr double kGravityM_S2 = 9.80665;
constexpr double kMolarMassAirKgPerMol = 0.0289644;
constexpr double kGasConstantJPerMolK = 8.3144598;
constexpr double kTroposphereTopM = 11000.0;

}  // namespace

double IsaPressurePa(double altitude_msl_m, double sea_level_pressure_pa) {
  if (!IsFinite(altitude_msl_m) || altitude_msl_m >= kTroposphereTopM ||
      !(sea_level_pressure_pa > 0.0)) {
    throw std::invalid_argument("ISA pressure: altitude/pressure out of range");
  }
  const double exponent = kGravityM_S2 * kMolarMassAirKgPerMol /
                          (kGasConstantJPerMolK * kLapseRateKPerM);
  return sea_level_pressure_pa *
         std::pow(1.0 - kLapseRateKPerM * altitude_msl_m / kSeaLevelTemperatureK,
                  exponent);
}

BaroModel::BaroModel(const BaroParams& params)
    : params_(params), engine_(params.seed) {
  if (!(params.sea_level_pressure_pa > 0.0) ||
      !IsFinite(params.ground_altitude_msl_m) ||
      !IsFinite(params.temperature_c) || params.temperature_c < -60.0 ||
      params.temperature_c > 60.0 || !IsFinite(params.noise_std_pa) ||
      params.noise_std_pa < 0.0) {
    throw std::invalid_argument("baro: invalid parameters");
  }
}

double BaroModel::PressureAtHeightPa(double height_above_origin_m) const {
  if (!IsFinite(height_above_origin_m)) {
    throw std::invalid_argument("baro: height must be finite");
  }
  // Ground pressure from ISA; above it, a troposphere whose temperature at
  // the ground is the configured air temperature (lapse rate as ISA).
  const double ground_pressure_pa = IsaPressurePa(
      params_.ground_altitude_msl_m, params_.sea_level_pressure_pa);
  const double ground_temperature_k = params_.temperature_c + 273.15;
  const double exponent = kGravityM_S2 * kMolarMassAirKgPerMol /
                          (kGasConstantJPerMolK * kLapseRateKPerM);
  return ground_pressure_pa *
         std::pow(1.0 - kLapseRateKPerM * height_above_origin_m /
                            ground_temperature_k,
                  exponent);
}

BaroSample BaroModel::Sample(double height_above_origin_m) {
  BaroSample sample;
  sample.pressure_pa = PressureAtHeightPa(height_above_origin_m) +
                       params_.noise_std_pa * unit_normal_(engine_);
  sample.temperature_c = params_.temperature_c;
  return sample;
}

}  // namespace pluto_x
