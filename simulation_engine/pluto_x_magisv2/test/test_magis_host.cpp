// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end checks of the production MagisV2 firmware running on the host.
// The firmware is a process-wide singleton, so the scenario is one test with
// sequential phases: boot -> calibrate -> arm -> attitude response.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "pluto_x_magisv2/magis_host.hpp"

namespace pluto_x_magisv2 {
namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kAccOneG = 4096.0;  // acc_1G for the ICM-20948 at +/-8 g
constexpr std::uint32_t kRcPeriodUs = 20000;  // 50 Hz MSP frames

constexpr std::uint16_t kCentre = 1500;
constexpr std::uint16_t kLow = 1000;
constexpr std::uint16_t kArmOn = 1500;     // AUX4 in [1300, 2100]
constexpr std::uint16_t kAux1Off = 2000;   // outside MAG and HEADFREE ranges

enum Motor { kRearRight = 0, kFrontRight = 1, kRearLeft = 2, kFrontLeft = 3 };

/// Sensor readings for a vehicle at rest with the given attitude, in the
/// PHYSICAL convention used by the test (roll + = right side down, pitch + =
/// NOSE UP, Z-Y-X), converted to what the MagisV2 drivers deliver:
/// accelerometer = specific force in FLU body,
///   f = g (sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)).
SensorInputs Tilted(double roll_deg, double pitch_up_deg) {
  SensorInputs s;
  const double r = roll_deg * kDegToRad;
  const double p = pitch_up_deg * kDegToRad;
  s.accel_counts = {static_cast<std::int16_t>(std::lround(kAccOneG * std::sin(p))),
                    static_cast<std::int16_t>(std::lround(kAccOneG * std::sin(r) * std::cos(p))),
                    static_cast<std::int16_t>(std::lround(kAccOneG * std::cos(r) * std::cos(p)))};
  // Earth field facing north, northern hemisphere: FLU (north, 0, -down).
  // The firmware maps AK09916 raw (x, y, z) to FLU (x, -y, -z).
  s.mag_counts = {200, 0, 400};
  s.battery_voltage_v = 4.1f;
  return s;
}

RcChannelsUs Rc(std::uint16_t throttle, std::uint16_t aux4) {
  // AETR1234: roll, pitch, throttle, yaw, AUX1, AUX2, AUX3, AUX4
  return {kCentre, kCentre, throttle, kCentre, kAux1Off, kLow, kLow, aux4};
}

constexpr double kGyroCountsPerDegS = 16.4;
constexpr std::uint32_t kSensorStepUs = 1000;

/// Rotates the vehicle from (roll0, pitch0) to (roll1, pitch1) degrees over
/// duration_us with a constant body rate, feeding the gyro rate and the
/// matching gravity vector every millisecond (small-angle rates), then holds.
void Rotate(MagisHost& fc, double roll0, double pitch0, double roll1,
            double pitch1, std::uint32_t duration_us, const RcChannelsUs& rc) {
  const double seconds = duration_us * 1e-6;
  const double roll_rate = (roll1 - roll0) / seconds;
  const double pitch_rate = (pitch1 - pitch0) / seconds;
  const std::uint32_t start = fc.time_us();
  std::uint32_t next_rc = start;
  while (fc.time_us() - start < duration_us) {
    const double f = static_cast<double>(fc.time_us() - start) / duration_us;
    SensorInputs s = Tilted(roll0 + f * (roll1 - roll0),
                            pitch0 + f * (pitch1 - pitch0));
    // FLU rates: roll rate about +x; nose-up is a NEGATIVE rotation about
    // +y (left).
    s.gyro_counts = {
        static_cast<std::int16_t>(std::lround(roll_rate * kGyroCountsPerDegS)),
        static_cast<std::int16_t>(std::lround(-pitch_rate * kGyroCountsPerDegS)),
        0};
    fc.SetSensors(s);
    if (fc.time_us() >= next_rc) {
      fc.SetRc(rc);
      next_rc += kRcPeriodUs;
    }
    fc.RunUntil(fc.time_us() + kSensorStepUs);
  }
  fc.SetSensors(Tilted(roll1, pitch1));
}

/// Runs for duration_us, delivering an RC frame every kRcPeriodUs.
void Fly(MagisHost& fc, std::uint32_t duration_us, const RcChannelsUs& rc) {
  const std::uint32_t end = fc.time_us() + duration_us;
  while (fc.time_us() < end) {
    fc.SetRc(rc);
    fc.RunUntil(fc.time_us() + kRcPeriodUs);
  }
}

TEST(MagisHost, BootCalibrateArmAndAttitudeResponse) {
  MagisHost& fc = MagisHost::Instance();
  fc.Initialise(0, Tilted(0.0, 0.0));
  EXPECT_THROW(fc.Initialise(0, Tilted(0.0, 0.0)), std::logic_error);

  // Phase 1: power-up, level and still, disarmed.
  Fly(fc, 20000000, Rc(kLow, kLow));  // 20 s: gyro + baro calibration
  FirmwareStatus st = fc.Status();
  EXPECT_TRUE(st.calibrated);
  EXPECT_TRUE(st.ok_to_arm);
  EXPECT_FALSE(st.armed);
  EXPECT_TRUE(st.angle_mode);  // Rx_ESP: ANGLE on AUX4 900..2100, always
  EXPECT_NEAR(st.roll_decideg, 0, 5);
  EXPECT_NEAR(st.pitch_decideg, 0, 5);
  EXPECT_NEAR(st.battery_decivolts, 41, 1);
  for (std::uint16_t pwm : fc.MotorPwm()) {
    EXPECT_EQ(pwm, 1000);  // motor_disarmed (mincommand)
  }
  printf("[boot] calibrated=%d ok_to_arm=%d heading=%d alt=%d cm loops=%u delays=%u\n",
         st.calibrated, st.ok_to_arm, st.heading_deg, st.estimated_altitude_cm,
         st.loop_calls, st.delay_calls_after_init);

  // Phase 2: arm through AUX4 (as the Pluto app does over MSP), throttle mid.
  Fly(fc, 1000000, Rc(kLow, kArmOn));
  EXPECT_TRUE(fc.Status().armed);
  Fly(fc, 1000000, Rc(kCentre, kArmOn));
  MotorPwmUs level = fc.MotorPwm();
  printf("[armed, level] M0..M3 = %u %u %u %u\n", level[0], level[1], level[2],
         level[3]);
  for (std::uint16_t pwm : level) {
    EXPECT_GT(pwm, 1100);
    EXPECT_LT(pwm, 2000);
  }

  // Phase 3: vehicle tilted 5 deg right (right side down). The estimator must
  // report +roll and the controller must lift the right side.
  Rotate(fc, 0.0, 0.0, 5.0, 0.0, 500000, Rc(kCentre, kArmOn));
  Fly(fc, 500000, Rc(kCentre, kArmOn));
  st = fc.Status();
  MotorPwmUs rolled = fc.MotorPwm();
  printf("[roll +5 deg] est roll=%d decideg, M0..M3 = %u %u %u %u\n",
         st.roll_decideg, rolled[0], rolled[1], rolled[2], rolled[3]);
  EXPECT_NEAR(st.roll_decideg, 50, 10);
  EXPECT_GT(rolled[kRearRight], rolled[kRearLeft]);
  EXPECT_GT(rolled[kFrontRight], rolled[kFrontLeft]);

  // Phase 4: vehicle 5 deg nose-up. The estimator must report pitch -5 deg
  // (MagisV2: + = nose down) and the controller must lift the rear.
  Rotate(fc, 5.0, 0.0, 0.0, 5.0, 500000, Rc(kCentre, kArmOn));
  Fly(fc, 500000, Rc(kCentre, kArmOn));
  st = fc.Status();
  MotorPwmUs pitched = fc.MotorPwm();
  printf("[pitch +5 deg] est pitch=%d decideg, M0..M3 = %u %u %u %u\n",
         st.pitch_decideg, pitched[0], pitched[1], pitched[2], pitched[3]);
  EXPECT_NEAR(st.pitch_decideg, -50, 10);
  EXPECT_GT(pitched[kRearRight], pitched[kFrontRight]);
  EXPECT_GT(pitched[kRearLeft], pitched[kFrontLeft]);

  // Phase 5: level again, then a counter-clockwise yaw rate (seen from
  // above; FLU +z) with the yaw stick centred. The rate loop must oppose it
  // with a clockwise reaction torque, which comes from speeding up the
  // rotors that spin counter-clockwise. The pair the firmware speeds up
  // therefore identifies MagisV2's propeller spin directions.
  Rotate(fc, 0.0, 5.0, 0.0, 0.0, 500000, Rc(kCentre, kArmOn));
  Fly(fc, 500000, Rc(kCentre, kArmOn));
  SensorInputs yawing = Tilted(0.0, 0.0);
  constexpr double kYawRateDegS = 30.0;
  yawing.gyro_counts[2] =
      static_cast<std::int16_t>(std::lround(kYawRateDegS * kGyroCountsPerDegS));
  fc.SetSensors(yawing);
  Fly(fc, 200000, Rc(kCentre, kArmOn));
  MotorPwmUs yawed = fc.MotorPwm();
  printf("[yaw CCW 30 deg/s] M0..M3 = %u %u %u %u\n", yawed[0], yawed[1],
         yawed[2], yawed[3]);
  // MagisV2 speeds up M1 (front-right) and M2 (rear-left): those are the
  // counter-clockwise propellers the firmware assumes. pluto_x_core's
  // magis_quad_x geometry encodes the same (quad_mixer.hpp); if this ever
  // changes, both must change together.
  EXPECT_GT(yawed[kFrontRight] + yawed[kRearLeft],
            yawed[kRearRight] + yawed[kFrontLeft]);
  fc.SetSensors(Tilted(0.0, 0.0));

  // Phase 6: disarm.
  Fly(fc, 1000000, Rc(kLow, kLow));
  EXPECT_FALSE(fc.Status().armed);
}

}  // namespace
}  // namespace pluto_x_magisv2
