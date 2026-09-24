// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// MagisHost: runs the production MagisV2 flight-controller firmware on the
// PC, driven by simulated time, sensors, battery and RC input.
//
// What runs: the firmware's own configuration defaults (resetConf with the
// Pluto X receiver setup), sensor processing and calibration, DCM attitude
// estimator, RC processing (rates/expo, arming, flight-mode switches), angle
// mode, barometric altitude hold, PID controller (pidRewrite), QUADX mixer,
// battery management and failsafe: mw.cpp loop() itself, unmodified. What is
// replaced: the hardware driver layer (see host/hal_*.cpp), and main.cpp's
// init(), whose flight-logic calls are made by Initialise() in the same order
// (hardware bring-up is skipped; see the list in magis_host.cpp).
//
// Busy loop: on the MCU loop() runs continuously and executes the control
// step every masterConfig.looptime (3500 us); between control steps it
// services RX and round-robin periodic tasks (baro, compass, altitude
// estimate). RunUntil() reproduces this by calling loop() every
// busy_loop_period_us of simulated time.
//
// Singleton: the firmware keeps its state in global variables, so there is
// exactly one instance per process and Initialise() may be called once.
//
// Firmware frame conventions (Cleanflight lineage, derived from
// flight/imu.cpp and pid.cpp and verified by test/test_magis_host.cpp):
// sensor/body frame FLU (x forward, y left, z up), world frame z up; roll
// positive = right side down, PITCH POSITIVE = NOSE DOWN, yaw positive =
// counter-clockwise seen from above (heading = -yaw).
//
// Units and frames of the inputs are those of the replaced drivers:
//   gyro   FLU body rates, 16.4 LSB per deg/s (ICM-20948 at +/-2000 dps)
//   accel  specific force in FLU body, 4096 LSB per g: level and at rest
//          reads (0, 0, +4096); free fall reads 0
//   mag    AK09916 raw axes, i.e. BEFORE the firmware's CW180_DEG_FLIP
//          alignment (the firmware turns raw (x, y, z) into FLU (x, -y, -z))
//   RC     8 channels in microseconds, AETR1234 order (roll, pitch, throttle,
//          yaw, AUX1..AUX4), as MSP_SET_RAW_RC carries them.
//   motors PWM in microseconds 1000..2000 for M0..M3 (rear-right,
//          front-right, rear-left, front-left).

#ifndef PLUTO_X_MAGISV2_MAGIS_HOST_HPP_
#define PLUTO_X_MAGISV2_MAGIS_HOST_HPP_

#include <array>
#include <cstdint>

namespace pluto_x_magisv2 {

inline constexpr int kRcChannelCount = 8;
inline constexpr int kMotorCount = 4;

struct SensorInputs {
  std::array<std::int16_t, 3> gyro_counts{};
  std::array<std::int16_t, 3> accel_counts{};
  std::array<std::int16_t, 3> mag_counts{};
  float baro_pressure_pa{101325.0f};
  float baro_temperature_c{25.0f};
  float battery_voltage_v{4.2f};
  float battery_current_a{0.0f};
};

using RcChannelsUs = std::array<std::uint16_t, kRcChannelCount>;
using MotorPwmUs = std::array<std::uint16_t, kMotorCount>;

/// Snapshot of firmware state, read from its globals.
struct FirmwareStatus {
  bool armed{false};
  bool ok_to_arm{false};
  bool calibrated{false};       ///< mw.cpp isCalibrated
  bool small_angle{false};
  bool angle_mode{false};
  bool baro_mode{false};        ///< altitude hold active
  bool mag_mode{false};
  bool headfree_mode{false};
  std::int16_t roll_decideg{0};   ///< estimator roll, + = right side down
  std::int16_t pitch_decideg{0};  ///< estimator pitch, + = NOSE DOWN
  std::int16_t heading_deg{0};    ///< 0..359
  std::int32_t estimated_altitude_cm{0};
  std::uint16_t battery_decivolts{0};  ///< firmware's filtered vBatRaw
  std::uint32_t loop_calls{0};
  std::uint32_t delay_calls_after_init{0};
};

struct MagisHostOptions {
  /// Simulated time between consecutive loop() calls (MCU busy loop).
  std::uint32_t busy_loop_period_us{100};
  /// Store a completed compass calibration in the configuration before the
  /// sensors start, as on a Pluto X whose compass has been calibrated once.
  /// The target defines MAG_ENFORCE: without a stored calibration
  /// (masterConfig.magScale != 0) the firmware stays "calibrating" and
  /// refuses to arm. The stored values are what the firmware's own
  /// calibration (sensors/compass.cpp) computes for an ideal sensor:
  /// magZero = 0, magScale = 10 on every axis.
  bool preload_mag_calibration{true};
};

class MagisHost {
 public:
  static MagisHost& Instance();

  MagisHost(const MagisHost&) = delete;
  MagisHost& operator=(const MagisHost&) = delete;

  /// Boots the firmware at time_us with the given sensor readings present
  /// (they are read during sensor detection and calibration start).
  /// Throws std::logic_error if called twice, std::runtime_error if the
  /// firmware halts (failureMode) during start-up.
  void Initialise(std::uint32_t time_us, const SensorInputs& initial_sensors,
                  const MagisHostOptions& options = {});

  /// Latest sensor readings; the firmware samples them when it reads its
  /// sensors inside loop().
  void SetSensors(const SensorInputs& inputs);

  /// Delivers one RC frame, as the MSP_SET_RAW_RC handler does
  /// (rxMspFrameReceive). Values are clamped to [0, 65535] by type.
  void SetRc(const RcChannelsUs& channels_us);

  /// Advances simulated time to time_us, calling loop() every
  /// busy_loop_period_us. Throws std::runtime_error if the firmware halts.
  /// Precondition (checked, std::invalid_argument): time does not decrease.
  void RunUntil(std::uint32_t time_us);

  /// Last PWM value written to each motor (M0..M3).
  MotorPwmUs MotorPwm() const;

  FirmwareStatus Status() const;

  bool initialised() const { return initialised_; }
  std::uint32_t time_us() const;

 private:
  MagisHost() = default;

  bool initialised_{false};
  MagisHostOptions options_;
  std::uint32_t loop_calls_{0};
};

}  // namespace pluto_x_magisv2

#endif  // PLUTO_X_MAGISV2_MAGIS_HOST_HPP_
