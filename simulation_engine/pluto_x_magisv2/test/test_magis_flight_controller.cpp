// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Closed loop: production MagisV2 firmware (MagisFlightController) flying
// the simulated Pluto X (SimulatedVehicle + legacy reference integrator).
// One scenario (the firmware is a per-process singleton).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>

#include "pluto_x/common/frames.hpp"
#include "pluto_x/config/config_loader.hpp"
#include "pluto_x/dynamics/legacy_dynamics.hpp"
#include "pluto_x/simulation/simulated_vehicle.hpp"
#include "pluto_x_magisv2/magis_flight_controller.hpp"

namespace pluto_x_magisv2 {
namespace {

constexpr std::int64_t kStepNs = 1000000;          // 1 ms physics
constexpr std::int64_t kRcPeriodNs = 20000000;     // 50 Hz MSP frames
constexpr double kRadToDeg = 180.0 / M_PI;

constexpr std::uint16_t kCentre = 1500;
constexpr std::uint16_t kLow = 1000;
constexpr std::uint16_t kArmOn = 1500;    // AUX4 in [1300, 2100]
constexpr std::uint16_t kAux1Off = 2000;  // outside MAG and HEADFREE ranges

pluto_x::RcFrame Rc(std::uint16_t roll, std::uint16_t pitch,
                    std::uint16_t throttle, std::uint16_t yaw,
                    std::uint16_t aux3, std::uint16_t aux4) {
  pluto_x::RcFrame frame;
  frame.channels_us = {roll, pitch, throttle, yaw, kAux1Off, kLow, aux3, aux4};
  return frame;
}

class ClosedLoop {
 public:
  explicit ClosedLoop(const pluto_x::LegacyStackConfig& config)
      : integrator_(config.vehicle, config.reference_state_clamps) {
    auto fc = std::make_unique<MagisFlightController>(config);
    fc_ = fc.get();
    vehicle_ = std::make_unique<pluto_x::SimulatedVehicle>(config, std::move(fc));
  }

  /// Flies for duration_s with the given sticks, sending a new RC frame
  /// every kRcPeriodNs.
  void Fly(double duration_s, pluto_x::RcFrame rc, bool print = false,
           std::int64_t print_every_ns = 250000000) {
    const std::int64_t end = t_ns_ + static_cast<std::int64_t>(duration_s * 1e9);
    while (t_ns_ < end) {
      if (t_ns_ % kRcPeriodNs == 0) {
        rc.sequence = ++sequence_;
        latest_rc_ = rc;
      }
      const auto out = vehicle_->Step(t_ns_, kStepNs, truth_, latest_rc_);
      ASSERT_TRUE(out.controller.healthy);
      last_ = out.controller;
      const pluto_x::Vector3 v0 = truth_.state.velocity_ned_m_s;
      integrator_.Step(truth_.state, out.propulsion.rotors.achieved,
                       out.propulsion.rotors.net_rotor_speed_rad_s, 1e-3,
                       pluto_x::frames::SwapNedEnu(out.wind_enu_m_s),
                       out.propulsion.rotors.rotor_speed_sum_rad_s);
      truth_.acceleration_ned_m_s2 = (truth_.state.velocity_ned_m_s - v0) / 1e-3;
      ASSERT_TRUE(pluto_x::IsFinite(truth_.state));
      t_ns_ += kStepNs;
      if (print && t_ns_ % print_every_ns == 0) {
        Print(out);
      }
    }
  }

  void Print(const pluto_x::SimulatedVehicleStep& out) const {
    const auto& s = truth_.state;
    const auto st = fc_->status();
    const auto pwm = fc_->motor_pwm_us();
    std::printf(
        "t=%6.2f s z_up=%6.3f m vz_up=%6.3f m/s rpy=(%5.1f %5.1f %6.1f) deg "
        "fw(roll,pitch,hdg)=(%4d %4d %3d) fw_alt=%5d cm armed=%d "
        "pwm=%u %u %u %u us V=%.2f T=%.3f N\n",
        t_ns_ * 1e-9, -s.position_ned_m.z(), -s.velocity_ned_m_s.z(),
        s.attitude.roll_rad * kRadToDeg, s.attitude.pitch_rad * kRadToDeg,
        s.attitude.yaw_rad * kRadToDeg, st.roll_decideg, st.pitch_decideg,
        st.heading_deg, st.estimated_altitude_cm, st.armed, pwm[0], pwm[1],
        pwm[2], pwm[3], out.propulsion.battery_voltage_v,
        out.propulsion.rotors.achieved.thrust_n);
  }

  const pluto_x::VehicleStateNed& state() const { return truth_.state; }
  const MagisFlightController& fc() const { return *fc_; }
  const pluto_x::FlightControllerOutput& last() const { return last_; }

 private:
  pluto_x::LegacyReferenceIntegrator integrator_;
  MagisFlightController* fc_{nullptr};
  std::unique_ptr<pluto_x::SimulatedVehicle> vehicle_;
  pluto_x::VehicleTruth truth_;
  std::optional<pluto_x::RcFrame> latest_rc_;
  std::uint64_t sequence_{0};
  std::int64_t t_ns_{0};
  pluto_x::FlightControllerOutput last_;
};

double Deg(double rad) { return rad * kRadToDeg; }

TEST(MagisFlightController, BootsArmsAndFlies) {
  pluto_x::LegacyStackConfig config =
      pluto_x::LoadLegacyStackConfigFile(PLUTO_X_ESTIMATED_CONFIG);
  config.wind.enabled = false;
  // The legacy reference-integrator clamps (50 deg/s, 45 deg) would distort
  // the firmware's rate loop; the Gazebo vehicle has none.
  config.reference_state_clamps.body_rate_limit_rad_s =
      pluto_x::Vector3(1e3, 1e3, 1e3);
  config.reference_state_clamps.attitude_limit_rad = {3.1, 1.5, 1e3};
  ASSERT_EQ(config.flight_controller.type,
            pluto_x::FlightControllerType::kMagisV2);
  ClosedLoop sim(config);
  EXPECT_THROW(MagisFlightController second(config), std::logic_error);

  // Power-up on the ground, disarmed: sensor calibration.
  sim.Fly(20.0, Rc(kCentre, kCentre, kLow, kCentre, kLow, kLow));
  EXPECT_TRUE(sim.fc().status().calibrated);
  EXPECT_TRUE(sim.fc().status().ok_to_arm);
  EXPECT_FALSE(sim.fc().status().armed);
  for (std::uint16_t pwm : sim.fc().motor_pwm_us()) EXPECT_EQ(pwm, 1000);

  // Arm (AUX4), then mid throttle: below hover, stays on the ground with
  // the heading held (regression: rotor-inertia yaw limit cycle).
  sim.Fly(1.0, Rc(kCentre, kCentre, kLow, kCentre, kLow, kArmOn));
  EXPECT_TRUE(sim.fc().status().armed);
  sim.Fly(4.0, Rc(kCentre, kCentre, kCentre, kCentre, kLow, kArmOn), true,
              1000000000);
  EXPECT_NEAR(sim.state().position_ned_m.z(), 0.0, 1e-9);
  EXPECT_LT(std::abs(Deg(sim.state().attitude.yaw_rad)), 2.0);
  const MotorPwmUs idle = sim.fc().motor_pwm_us();
  EXPECT_LT(*std::max_element(idle.begin(), idle.end()) -
                *std::min_element(idle.begin(), idle.end()),
            30);

  // Throttle above hover: vertical climb, attitude and heading held.
  sim.Fly(3.0, Rc(kCentre, kCentre, 1800, kCentre, kLow, kArmOn), true,
          500000000);
  const pluto_x::VehicleStateNed climb = sim.state();
  EXPECT_GT(-climb.position_ned_m.z(), 3.0);
  EXPECT_LT(std::abs(Deg(climb.attitude.roll_rad)), 3.0);
  EXPECT_LT(std::abs(Deg(climb.attitude.pitch_rad)), 3.0);
  EXPECT_LT(std::abs(Deg(climb.attitude.yaw_rad)), 3.0);
  EXPECT_LT(climb.position_ned_m.head<2>().norm(), 0.5);
  // The firmware's altitude estimate follows (loose: baro + accelerometer
  // fusion; see docs).
  EXPECT_NEAR(sim.fc().status().estimated_altitude_cm * 0.01,
              -climb.position_ned_m.z(), 0.25 * -climb.position_ned_m.z());

  // Roll right: the vehicle must bank right (+roll FRD) and drift +east.
  sim.Fly(1.0, Rc(1600, kCentre, 1760, kCentre, kLow, kArmOn), true,
          250000000);
  EXPECT_GT(Deg(sim.state().attitude.roll_rad), 3.0);
  EXPECT_GT(sim.state().velocity_ned_m_s.y(), 0.3);
  // Pitch forward (stick up = nose down): the vehicle accelerates north.
  sim.Fly(1.0, Rc(kCentre, 1600, 1760, kCentre, kLow, kArmOn), true,
          250000000);
  EXPECT_LT(Deg(sim.state().attitude.pitch_rad), -3.0);  // FRD: nose down < 0
  // Telemetry in physical conventions tracks the truth (loosely).
  EXPECT_LT(sim.last().telemetry.pitch_deg, -2.0);
  EXPECT_TRUE(sim.last().telemetry.armed);
  EXPECT_NEAR(sim.last().telemetry.battery_v, 3.9, 0.3);
  EXPECT_GT(sim.state().velocity_ned_m_s.x(), 0.3);
  // Yaw right: heading increases clockwise (+yaw FRD).
  const double yaw0 = sim.state().attitude.yaw_rad;
  sim.Fly(1.0, Rc(kCentre, kCentre, 1760, 1600, kLow, kArmOn), true,
          250000000);
  EXPECT_GT(Deg(sim.state().attitude.yaw_rad - yaw0), 10.0);

  // Disarm: motors stop.
  sim.Fly(0.5, Rc(kCentre, kCentre, kLow, kCentre, kLow, kLow));
  EXPECT_FALSE(sim.fc().status().armed);
  for (std::uint16_t pwm : sim.fc().motor_pwm_us()) EXPECT_EQ(pwm, 1000);
  EXPECT_FALSE(sim.fc().halted());
}

}  // namespace
}  // namespace pluto_x_magisv2
