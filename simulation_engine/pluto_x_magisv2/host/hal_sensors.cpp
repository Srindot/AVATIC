// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Replaces the Pluto X sensor drivers (drivers/accgyro_mpu.cpp,
// accgyro_icm20948.cpp, compass_ak09916.cpp, barometer_icp10111.cpp).
//
// Each detect function reproduces the non-I2C side effects of the real one
// (function pointers, gyro->scale, acc_1G, the ICP10111 measurement timing)
// and reads its samples from HostState instead of the I2C bus. Everything
// above the driver (sensors/*.cpp alignment, calibration, filtering, the DCM
// estimator, compass and barometer processing) is production code.

#include <cstring>

#include "host_state.hpp"

#include "firmware_includes.hpp"

using pluto_x_magisv2::host::State;

// Defined in drivers/accgyro_mpu.cpp on the MCU.
mpuDetectionResult_t mpuDetectionResult;

namespace {

// --- ICM-20948 gyro / accelerometer --------------------------------------

void HostGyroInit(uint16_t /*lpf*/) {}

bool HostGyroRead(int16_t* data) {
  std::memcpy(data, State().gyro_counts.data(), 3 * sizeof(int16_t));
  return true;
}

// Real icm20948AccInit: mpuIntExtiInit(); acc_1G = 512 * 8.
void HostAccInit(void) { acc_1G = 512 * 8; }

bool HostAccRead(int16_t* data) {
  std::memcpy(data, State().accel_counts.data(), 3 * sizeof(int16_t));
  return true;
}

// --- AK09916 magnetometer --------------------------------------------------

void HostMagInit(void) {}

bool HostMagRead(int16_t* data) {
  std::memcpy(data, State().mag_counts.data(), 3 * sizeof(int16_t));
  return true;
}

// --- ICP10111 barometer ----------------------------------------------------
// Same conversion timing as the real driver: measurment_start(mode) records
// the start time (ms) and duration; read() returns false until it elapsed.

uint32_t g_baro_start_ms = 0;
uint32_t g_baro_duration_ms = 7;

uint32_t HostBaroMeasureStart(mmode mode) {
  switch (mode) {
    case FAST:
      g_baro_duration_ms = 3;
      break;
    case ACCURATE:
      g_baro_duration_ms = 24;
      break;
    case VERY_ACCURATE:
      g_baro_duration_ms = 98;
      break;
    case NORMAL:
    default:
      g_baro_duration_ms = 7;
      break;
  }
  g_baro_start_ms = millis();
  return g_baro_duration_ms;
}

bool HostBaroRead(uint32_t current_time_ms, float* pressure,
                  float* temperature) {
  if (pressure == nullptr || temperature == nullptr) {
    return false;
  }
  if ((current_time_ms - g_baro_start_ms) < g_baro_duration_ms) {
    return false;
  }
  *pressure = State().baro_pressure_pa;
  *temperature = State().baro_temperature_c;
  return true;
}

}  // namespace

mpuDetectionResult_t* detectMpu(const extiConfig_t* /*configToUse*/) {
  std::memset(&mpuDetectionResult, 0, sizeof(mpuDetectionResult));
  mpuDetectionResult.sensor = MPU_ICM_20948;
  return &mpuDetectionResult;
}

bool icm20948GyroDetect(gyro_t* gyro) {
  if (mpuDetectionResult.sensor != MPU_ICM_20948) {
    return false;
  }
  gyro->init = HostGyroInit;
  gyro->read = HostGyroRead;
  gyro->scale = 1.0f / 131.0f;  // as the real driver sets it
  return true;
}

bool icm20948AccDetect(acc_t* acc) {
  if (mpuDetectionResult.sensor != MPU_ICM_20948) {
    return false;
  }
  acc->init = HostAccInit;
  acc->read = HostAccRead;
  return true;
}

bool ak09916Detect(mag_t* mag) {
  mag->init = HostMagInit;
  mag->read = HostMagRead;
  return true;
}

bool icp10111Detect(baro_t* baro) {
  baro->measurment_start = HostBaroMeasureStart;
  baro->read = HostBaroRead;
  HostBaroMeasureStart(NORMAL);  // real driver starts the first measurement
  return true;
}
