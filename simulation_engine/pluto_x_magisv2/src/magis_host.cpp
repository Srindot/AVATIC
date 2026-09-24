// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later

#include "pluto_x_magisv2/magis_host.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "host_state.hpp"

#include "firmware_includes.hpp"

// Defined in mw.cpp (no header declaration).
extern bool motorControlEnable;
extern uint32_t previousTime;
// Defined in sensors/battery.cpp (battery voltage, 0.1 V steps, filtered).
extern uint16_t vBatRaw;

namespace pluto_x_magisv2 {
namespace {

namespace host = pluto_x_magisv2::host;

/// INA219_SHUNT_RESISTOR_MILLIOHM in drivers/ina219.h. sensors/battery.cpp
/// converts shunt readings with mA = (shunt / 10) / this value.
constexpr float kFirmwareShuntMilliohm = 0.002f;
constexpr float kMilliampsPerAmp = 1000.0f;
constexpr float kDecivoltsPerVolt = 10.0f;

template <typename T>
T SaturatingCast(float value) {
  const float lo = static_cast<float>(std::numeric_limits<T>::min());
  const float hi = static_cast<float>(std::numeric_limits<T>::max());
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<T>(std::lround(std::fmin(std::fmax(value, lo), hi)));
}

void StoreSensors(const SensorInputs& s) {
  host::HostState& state = host::State();
  state.gyro_counts = s.gyro_counts;
  state.accel_counts = s.accel_counts;
  state.mag_counts = s.mag_counts;
  state.baro_pressure_pa = s.baro_pressure_pa;
  state.baro_temperature_c = s.baro_temperature_c;
  state.bus_voltage_decivolts =
      SaturatingCast<std::uint16_t>(s.battery_voltage_v * kDecivoltsPerVolt);
  // Inverse of the firmware's conversion (see kFirmwareShuntMilliohm).
  state.shunt_voltage_mv = SaturatingCast<std::int16_t>(
      s.battery_current_a * kMilliampsPerAmp * kFirmwareShuntMilliohm * 10.0f);
}

}  // namespace

MagisHost& MagisHost::Instance() {
  static MagisHost instance;
  return instance;
}

std::uint32_t MagisHost::time_us() const { return host::State().time_us; }

// Mirrors main.cpp init(). Calls marked [skipped] configure STM32 hardware
// that has no host counterpart; every other call is made, in init()'s order:
//   printfSupportInit            [skipped] serial printf
//   initEEPROM / ensureEEPROMContainsValidData / readEEPROM
//   [host] optional stored compass calibration (MagisHostOptions)
//   SCB->CPACR, SetSysClock, systemInit, ledInit, ESP GPIO   [skipped]
//   latchActiveFeatures
//   serialInit                   [skipped] UART ports (RC is injected)
//   mixerInit
//   motorControlEnable = true (no ONESHOT)
//   beeperInit, spiInit, i2cInit, adcInit                    [skipped]
//   initBoardAlignment
//   sensorsAutodetectmpu, sensorsAutodetectbaro, checkBaroDriftDuringStartup
//   INA219_Init                  [skipped] battery is simulated directly
//   LED / beeper start-up blink (15 x 50 ms delay)           [skipped]
//   compassInit (if mag), imuInit
//   mspInit, cliInit             [skipped] serial protocols
//   failsafeInit, rxInit
//   previousTime = micros(), updateGains, gyroSetCalibrationCycles,
//   baroSetCalibrationCycles, baroInit, baroCalibrate
//   timerStart                   [skipped]
//   ENABLE_STATE(SMALL_ANGLE), DISABLE_ARMING_FLAG(PREVENT_ARMING)
//   batteryInit (if INA219 features)
//   latchActiveFeatures, motorControlEnable = true
//   ranging / optic-flow init    [skipped] not fitted / disabled
//   updatePosGains, resetUser, plutoInit
//   timerDataConfiguration, reverseMotorGPIOInit, timerInit  [skipped]
//   pwmRxInit, pwmInit           [skipped]; mixerUsePWMOutputConfiguration
//                                is called with the 4 motors pwmInit reports
//   OLED / ADC / X-ranging / UWB init                        [skipped]
void MagisHost::Initialise(std::uint32_t time_us,
                           const SensorInputs& initial_sensors,
                           const MagisHostOptions& options) {
  if (initialised_) {
    throw std::logic_error("MagisHost::Initialise called twice");
  }
  if (options.busy_loop_period_us == 0) {
    throw std::invalid_argument("busy_loop_period_us must be > 0");
  }
  options_ = options;
  host::State().time_us = time_us;
  StoreSensors(initial_sensors);

  initEEPROM();
  ensureEEPROMContainsValidData();
  readEEPROM();
  if (options.preload_mag_calibration) {
    // See MagisHostOptions::preload_mag_calibration. Values are those
    // sensors/compass.cpp derives from equal min/max ranges on all axes.
    constexpr std::int16_t kIdealMagScale = 10;
    for (int axis = 0; axis < 3; ++axis) {
      masterConfig.magZero.raw[axis] = 0;
      masterConfig.magScale.raw[axis] = kIdealMagScale;
    }
    writeEEPROM();
    readEEPROM();
  }
  latchActiveFeatures();
  mixerInit(static_cast<mixerMode_e>(masterConfig.mixerMode),
            masterConfig.customMotorMixer);
  if (!feature(FEATURE_ONESHOT125)) {
    motorControlEnable = true;
  }
  initBoardAlignment(&masterConfig.boardAlignment);
  if (!sensorsAutodetectmpu(&masterConfig.sensorAlignmentConfig,
                            masterConfig.gyro_lpf, masterConfig.acc_hardware,
                            masterConfig.mag_hardware)) {
    throw std::runtime_error("MagisV2: accelerometer/gyro not detected");
  }
  if (!sensorsAutodetectbaro(masterConfig.baro_hardware)) {
    throw std::runtime_error("MagisV2: barometer not detected");
  }
  checkBaroDriftDuringStartup();
  if (sensors(SENSOR_MAG)) {
    compassInit();
  }
  imuInit();
  failsafeInit(&masterConfig.rxConfig,
               masterConfig.flight3DConfig.deadband3d_throttle);
  rxInit(&masterConfig.rxConfig, currentProfile->modeActivationConditions);
  previousTime = micros();
  updateGains();
  gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
  baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
  baroInit();
  baroCalibrate();
  ENABLE_STATE(SMALL_ANGLE);
  DISABLE_ARMING_FLAG(PREVENT_ARMING);
  if (feature(FEATURE_INA219_VBAT | FEATURE_INA219_CBAT)) {
    batteryInit(&masterConfig.batteryConfig);
  }
  latchActiveFeatures();
  motorControlEnable = true;
  updatePosGains();
  resetUser();
  plutoInit();
  pwmOutputConfiguration_t pwm_output = {};
  pwm_output.motorCount = kMotorCount;
  mixerUsePWMOutputConfiguration(&pwm_output);

  host::State().initialised = true;
  initialised_ = true;
}

void MagisHost::SetSensors(const SensorInputs& inputs) {
  StoreSensors(inputs);
}

void MagisHost::SetRc(const RcChannelsUs& channels_us) {
  if (!initialised_) {
    throw std::logic_error("MagisHost::SetRc before Initialise");
  }
  RcChannelsUs frame = channels_us;
  rxMspFrameReceive(frame.data(), kRcChannelCount);
}

void MagisHost::RunUntil(std::uint32_t target_time_us) {
  if (!initialised_) {
    throw std::logic_error("MagisHost::RunUntil before Initialise");
  }
  host::HostState& state = host::State();
  // Unsigned wrap-safe comparison, as the firmware itself uses.
  if (static_cast<std::int32_t>(target_time_us - state.time_us) < 0) {
    throw std::invalid_argument("MagisHost::RunUntil: time must not decrease");
  }
  while (static_cast<std::int32_t>(target_time_us - state.time_us) > 0) {
    const std::uint32_t remaining = target_time_us - state.time_us;
    state.time_us += remaining < options_.busy_loop_period_us
                         ? remaining
                         : options_.busy_loop_period_us;
    loop();
    ++loop_calls_;
  }
}

MotorPwmUs MagisHost::MotorPwm() const { return host::State().motor_pwm_us; }

FirmwareStatus MagisHost::Status() const {
  FirmwareStatus s;
  s.armed = ARMING_FLAG(ARMED);
  s.ok_to_arm = ARMING_FLAG(OK_TO_ARM);
  s.calibrated = isCalibrated;
  s.small_angle = STATE(SMALL_ANGLE);
  s.angle_mode = FLIGHT_MODE(ANGLE_MODE);
  s.baro_mode = FLIGHT_MODE(BARO_MODE);
  s.mag_mode = FLIGHT_MODE(MAG_MODE);
  s.headfree_mode = FLIGHT_MODE(HEADFREE_MODE);
  s.roll_decideg = inclination.values.rollDeciDegrees;
  s.pitch_decideg = inclination.values.pitchDeciDegrees;
  s.heading_deg = heading;
  s.estimated_altitude_cm = altitudeHoldGetEstimatedAltitude();
  s.battery_decivolts = vBatRaw;
  s.loop_calls = loop_calls_;
  s.delay_calls_after_init = host::State().delay_calls_after_init;
  return s;
}

}  // namespace pluto_x_magisv2
