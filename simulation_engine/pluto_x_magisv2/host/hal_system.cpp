// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Replaces drivers/system.c (SysTick time base, delays, reset, failure mode).
//
// Time comes from the simulator (MagisHost advances HostState::time_us), so
// every time-dependent firmware computation (loop time, DCM deltaT, PID
// cycleTime, baro conversion timing, RC timeouts) runs on simulation time.

#include <sstream>

#include "host_state.hpp"

#include "firmware_includes.hpp"

namespace pluto_x_magisv2::host {

HostState& State() {
  static HostState state;
  return state;
}

}  // namespace pluto_x_magisv2::host

using pluto_x_magisv2::host::FirmwareHalt;
using pluto_x_magisv2::host::State;

uint32_t micros(void) { return State().time_us; }

uint32_t millis(void) { return State().time_us / 1000u; }

// On the MCU these busy-wait while time passes. During start-up
// (MagisHost::Initialise) they advance the firmware clock the same way, so
// timed start-up sequences work as on the MCU: e.g. icp10111BaroCalibrate()
// waits with delay() for baro conversions and would otherwise never see one
// complete (ground pressure left at 0). MagisHost keeps the resulting boot
// time as an offset between the firmware clock and the simulation clock.
// After initialisation they return immediately (time only advances with the
// simulator) and are counted, so that any use in the flight loop is visible.
void delay(uint32_t ms) {
  if (State().initialised) {
    ++State().delay_calls_after_init;
  } else {
    State().time_us += ms * 1000u;
  }
}

void delayMicroseconds(uint32_t us) {
  if (State().initialised) {
    ++State().delay_calls_after_init;
  } else {
    State().time_us += us;
  }
}

void failureMode(uint16_t mode) {
  std::ostringstream text;
  text << "MagisV2 entered failureMode(" << mode << ")";
  throw FirmwareHalt(text.str());
}

void systemReset(void) { throw FirmwareHalt("MagisV2 requested systemReset()"); }

void systemResetToBootloader(void) {
  throw FirmwareHalt("MagisV2 requested systemResetToBootloader()");
}
