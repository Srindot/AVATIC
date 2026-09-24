// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include "pluto_x/config/config_loader.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

std::string ReadConfigText(const char* path) {
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string ReadDefaultConfigText() {
  return ReadConfigText(PLUTO_X_DEFAULT_CONFIG);
}

/// Returns the message of the ConfigError thrown for `yaml`, or "" if none.
std::string ErrorFor(const std::string& yaml) {
  try {
    LoadLegacyStackConfigString(yaml);
  } catch (const ConfigError& error) {
    return error.what();
  }
  return "";
}

std::string Replace(const std::string& text, const std::string& pattern,
                    const std::string& replacement) {
  return std::regex_replace(text, std::regex(pattern), replacement,
                            std::regex_constants::format_first_only);
}

TEST(ConfigLoader, LoadsShippedLegacyValues) {
  const LegacyStackConfig c = test::LoadDefaultConfig();
  EXPECT_DOUBLE_EQ(c.vehicle.mass_kg, 1.4);
  EXPECT_DOUBLE_EQ(c.vehicle.arm_length_m, 0.56);
  EXPECT_DOUBLE_EQ(c.vehicle.inertia_diag_kg_m2.z(), 0.10);
  EXPECT_DOUBLE_EQ(c.rotor.thrust_coefficient_n_s2, 0.000013328);
  EXPECT_DOUBLE_EQ(c.rotor.speed_squared_max_rad2_s2, 855625.0);
  EXPECT_DOUBLE_EQ(c.actuation_limits.thrust_max_n, 43.5);
  EXPECT_DOUBLE_EQ(c.controller.period_s, 0.01);
  EXPECT_DOUBLE_EQ(c.controller.position.down.kd, -5.05);
  EXPECT_DOUBLE_EQ(c.controller.rate.yaw.integral_window, 0.174532);
  EXPECT_EQ(c.pilot.initial_mode, FlightMode::kPositionHold);
  EXPECT_DOUBLE_EQ(c.reference_state_clamps.attitude_limit_rad.yaw_rad, 10.0);
  EXPECT_FALSE(DescribeLegacyStackConfig(c).empty());
}

TEST(ConfigLoader, MissingKeyReportsFullPath) {
  const std::string yaml =
      Replace(ReadDefaultConfigText(), "mass_kg: 1\\.4", "mass_typo: 1.4");
  EXPECT_NE(ErrorFor(yaml).find("vehicle.mass_kg"), std::string::npos);
}

TEST(ConfigLoader, RejectsNonNumericAndNonFinite) {
  const std::string text = ReadDefaultConfigText();
  EXPECT_NE(ErrorFor(Replace(text, "mass_kg: 1\\.4", "mass_kg: heavy"))
                .find("must be a number"),
            std::string::npos);
  EXPECT_NE(ErrorFor(Replace(text, "mass_kg: 1\\.4", "mass_kg: .nan"))
                .find("must be finite"),
            std::string::npos);
}

TEST(ConfigLoader, RejectsOutOfRangeValues) {
  const std::string text = ReadDefaultConfigText();
  EXPECT_NE(ErrorFor(Replace(text, "mass_kg: 1\\.4", "mass_kg: -1.4")),
            "");
  EXPECT_NE(ErrorFor(Replace(text, "max: 43\\.5", "max: -1.0")), "");
  EXPECT_NE(ErrorFor(Replace(text, "period_s: 0\\.01", "period_s: 0")), "");
  EXPECT_NE(ErrorFor(Replace(text, "hover_thrust_n: 13\\.734",
                             "hover_thrust_n: 100.0")),
            "");
  EXPECT_NE(ErrorFor(Replace(text, "initial_mode: position_hold",
                             "initial_mode: acro")),
            "");
  EXPECT_NE(ErrorFor(Replace(text, "error_frame: yaw_only",
                             "error_frame: body")),
            "");
}

TEST(ConfigLoader, RejectsNonPhysicalLegacyInertia) {
  // kwad.cpp's jz = 0.24 with jx = jy = 0.05 is not a rigid body.
  const std::string yaml = Replace(ReadDefaultConfigText(),
                                   "zz: 0\\.10", "zz: 0.24");
  EXPECT_NE(ErrorFor(yaml).find("triangle inequality"), std::string::npos);
}

TEST(ConfigLoader, ParsesLegacyErrorFrame) {
  const LegacyStackConfig c = LoadLegacyStackConfigString(
      Replace(ReadDefaultConfigText(), "error_frame: yaw_only",
              "error_frame: legacy_full_attitude"));
  EXPECT_EQ(c.controller.position.error_frame,
            PositionErrorFrame::kLegacyFullAttitude);
}

TEST(ConfigLoader, LoadsEstimatedPlutoX) {
  const LegacyStackConfig c = test::LoadPlutoXConfig();
  EXPECT_DOUBLE_EQ(c.vehicle.mass_kg, 0.068);  // 60 g + 8 g camera
  EXPECT_FALSE(c.vehicle.legacy_double_arm_torque);
  EXPECT_EQ(c.rotor.layout, RotorLayout::kMagisQuadX);
  EXPECT_NEAR(c.pilot.hover_thrust_n, c.vehicle.mass_kg * c.vehicle.gravity_m_s2,
              1e-4);
  EXPECT_NEAR(c.actuation_limits.thrust_max_n,
              4.0 * c.rotor.thrust_coefficient_n_s2 *
                  c.rotor.speed_squared_max_rad2_s2,
              1e-3);
}

TEST(ConfigLoader, RejectsSpinUpTorqueWithInstantaneousRotors) {
  const std::string yaml =
      Replace(ReadConfigText(PLUTO_X_ESTIMATED_CONFIG),
              "time_constant_s: 0\\.03", "time_constant_s: 0.0");
  EXPECT_NE(ErrorFor(yaml).find("spin_up_reaction_torque"), std::string::npos);
}

TEST(ConfigLoader, RejectsBadLayoutAndFlag) {
  const std::string text = ReadDefaultConfigText();
  EXPECT_NE(ErrorFor(Replace(text, "layout: legacy_plus", "layout: hexa")), "");
  EXPECT_NE(ErrorFor(Replace(text, "legacy_double_arm_torque: true",
                             "legacy_double_arm_torque: maybe")),
            "");
}

TEST(ConfigLoader, LoadsMagisV2Sensors) {
  const LegacyStackConfig c = test::LoadPlutoXConfig();
  ASSERT_EQ(c.flight_controller.type, FlightControllerType::kMagisV2);
  const MagisV2Params& m = c.flight_controller.magisv2;
  EXPECT_EQ(m.busy_loop_period_us, 100u);
  // 16.4 LSB per deg/s and 4096 LSB per g0
  EXPECT_NEAR(m.imu.gyro_counts_per_rad_s * 3.14159265358979 / 180.0, 16.4,
              1e-9);
  EXPECT_NEAR(m.imu.accel_counts_per_m_s2 * 9.80665, 4096.0, 1e-9);
  const Matrix3 frd_to_flu = Vector3(1.0, -1.0, -1.0).asDiagonal();
  EXPECT_EQ(m.imu.sensor_from_body, frd_to_flu);
  EXPECT_DOUBLE_EQ(m.magnetic_field_ned_ut.x(), 40.0);
  EXPECT_DOUBLE_EQ(m.baro.ground_altitude_msl_m, 505.0);
  EXPECT_NE(DescribeLegacyStackConfig(c).find("magisv2"), std::string::npos);
  // the legacy config needs no magisv2 section
  EXPECT_EQ(test::LoadDefaultConfig().flight_controller.type,
            FlightControllerType::kLegacy);
}

TEST(ConfigLoader, RejectsBadFlightController) {
  const std::string text = ReadConfigText(PLUTO_X_ESTIMATED_CONFIG);
  EXPECT_NE(ErrorFor(Replace(text, "type: magisv2", "type: px4"))
                .find("flight_controller.type"),
            std::string::npos);
  EXPECT_NE(ErrorFor(Replace(text, "busy_loop_period_us: 100",
                             "busy_loop_period_us: 5000"))
                .find("busy_loop_period_us"),
            std::string::npos);
  EXPECT_NE(ErrorFor(Replace(text, "mag_counts_per_ut: 6\\.667",
                             "mag_counts_per_ut: -1"))
                .find("mag_counts_per_ut"),
            std::string::npos);
  EXPECT_NE(ErrorFor(Replace(text, "layout: magis_quad_x", "layout: legacy_plus"))
                .find("requires rotor.layout"),
            std::string::npos);
  // type magisv2 requires the magisv2 section
  EXPECT_NE(ErrorFor(Replace(ReadDefaultConfigText(), "type: legacy",
                             "type: magisv2"))
                .find("flight_controller.magisv2"),
            std::string::npos);
}

TEST(ConfigLoader, OverridesFlightControllerType) {
  EXPECT_EQ(LoadLegacyStackConfigFile(PLUTO_X_ESTIMATED_CONFIG, "legacy")
                .flight_controller.type,
            FlightControllerType::kLegacy);
  EXPECT_THROW(LoadLegacyStackConfigFile(PLUTO_X_DEFAULT_CONFIG, "magisv2"),
               ConfigError);  // no magisv2 section, legacy_plus layout
  EXPECT_THROW(LoadLegacyStackConfigFile(PLUTO_X_ESTIMATED_CONFIG, "px4"),
               ConfigError);
}

TEST(ConfigLoader, ReportsUnreadableFile) {
  EXPECT_THROW(LoadLegacyStackConfigFile("/nonexistent/pluto.yaml"),
               ConfigError);
  EXPECT_THROW(LoadLegacyStackConfigString("[1, 2, 3]"), ConfigError);
}

}  // namespace
}  // namespace pluto_x
