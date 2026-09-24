// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// MagisFlightController: the production MagisV2 firmware (MagisHost) as a
// pluto_x::FlightController for the simulated vehicle.
//
// Per Step(input):
//   1. sensors   from the physical truth, in the firmware's conventions
//                (see magis_host.hpp):
//                  gyro   ImuModel, FRD body rate -> FLU counts
//                  accel  ImuModel, specific force R^T (a - g) -> FLU counts
//                  mag    field R^T B_ned in FRD, which equals the AK09916
//                         raw axes (the firmware maps raw (x, y, z) to FLU
//                         (x, -y, -z) = FRD -> FLU), times counts/uT
//                  baro   BaroModel (ISA) at the vehicle height
//                  power  battery terminal voltage and current
//   2. RC        a frame with a new sequence number is delivered via
//                MSP_SET_RAW_RC (MagisHost::SetRc)
//   3. firmware  MagisHost::RunUntil(t): mw.cpp loop() every
//                busy_loop_period_us; the firmware boots on the first Step
//   4. motors    duty = (PWM - 1000 us) / 1000 us, clamped to [0, 1]
//                (brushed motors: MagisV2 writes the PWM value as the duty of
//                a 1000-count timer period)
//
// The firmware keeps its state in globals (MagisHost is a singleton), so at
// most one MagisFlightController can exist per process (checked).
// Simulated time is passed to the firmware in microseconds as uint32_t, as
// micros() on the MCU; runs are limited to 4294 s (checked).
//
// If the firmware halts (failureMode/systemReset), the controller stays
// halted: motors off, healthy = false.

#ifndef PLUTO_X_MAGISV2_MAGIS_FLIGHT_CONTROLLER_HPP_
#define PLUTO_X_MAGISV2_MAGIS_FLIGHT_CONTROLLER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "pluto_x/config/legacy_params.hpp"
#include "pluto_x/control/flight_controller.hpp"
#include "pluto_x/sensors/baro_model.hpp"
#include "pluto_x/sensors/imu_model.hpp"
#include "pluto_x_magisv2/magis_host.hpp"

namespace pluto_x_magisv2 {

class MagisFlightController : public pluto_x::FlightController {
 public:
  /// Preconditions (checked, std::invalid_argument): config.flight_controller
  /// .type == kMagisV2. Throws std::logic_error if the firmware is already
  /// owned by another instance in this process.
  explicit MagisFlightController(const pluto_x::LegacyStackConfig& config);
  ~MagisFlightController() override;

  MagisFlightController(const MagisFlightController&) = delete;
  MagisFlightController& operator=(const MagisFlightController&) = delete;

  std::string Name() const override { return "magisv2_firmware"; }

  /// Preconditions (checked, std::invalid_argument): input.time_ns >= 0,
  /// non-decreasing, below 2^32 us.
  pluto_x::FlightControllerOutput Step(
      const pluto_x::FlightControllerInput& input) override;

  /// Firmware state after the most recent Step (for logging and tests).
  FirmwareStatus status() const { return status_; }
  MotorPwmUs motor_pwm_us() const { return pwm_us_; }
  bool halted() const { return halted_; }
  /// Last sensor inputs given to the firmware.
  const SensorInputs& last_sensors() const { return sensors_; }

 private:
  SensorInputs MakeSensors(const pluto_x::FlightControllerInput& input);

  pluto_x::MagisV2Params params_;
  double gravity_m_s2_;
  MagisHostOptions options_;
  pluto_x::ImuModel imu_;
  pluto_x::BaroModel baro_;
  std::optional<std::uint64_t> last_rc_sequence_;
  SensorInputs sensors_;
  FirmwareStatus status_;
  MotorPwmUs pwm_us_{};
  std::uint32_t last_loop_calls_{0};
  bool halted_{false};
};

}  // namespace pluto_x_magisv2

#endif  // PLUTO_X_MAGISV2_MAGIS_FLIGHT_CONTROLLER_HPP_
