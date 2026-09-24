// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Inert replacements for STM32 peripherals and on-board devices that have no
// counterpart in the simulation. Each one is listed with what it replaces and
// why doing nothing is correct on the host:
//
//   RCC_*, GPIO_*, gpioInit    clocks / pins. No physical pins exist.
//   ADC_*, DMA_*               user ADC API (API-Src/Peripheral-ADC.cpp).
//   SPI_*, spiInit/Transfer    SPI bus. Only the M25P16 blackbox flash and
//                              user SPI API use it.
//   i2cRead/Write/...          I2C bus. The flight sensors are simulated by
//                              hal_sensors.cpp; remaining users (user I2C
//                              API, OLED, ranging) get "bus error".
//   i2c_OLED_*, ug2864...      OLED display (drawing only).
//   multiWiiFont               OLED font table (read by io/oled_display.c).
//   m25p16_*                   SPI flash for blackbox logging: reports "not
//                              ready" so no log is written.
//   LaserSensor::*             VL53L0X time-of-flight ranging (not fitted in
//                              the default Pluto X configuration).
//   bodyRate, flowRate, ...    optical-flow globals (OPTIC_FLOW is disabled
//                              in the PRIMUSX2 target).
//   ppmRead/pwmRead/...        PPM/PWM receivers. The Pluto X uses MSP over
//                              Wi-Fi (FEATURE_RX_MSP), injected by MagisHost.
//   uartOpen                   UARTs. The ESP/Wi-Fi MSP link is replaced by
//                              direct RC injection; no serial port is opened.
//   ledOperator                status LEDs.

#include <cstdint>

#include "firmware_includes.hpp"

// --- clocks, pins, ADC, DMA, SPI (StdPeriph) --------------------------------
void RCC_ADCCLKConfig(uint32_t) {}
void RCC_AHBPeriphClockCmd(uint32_t, FunctionalState) {}
void RCC_APB1PeriphClockCmd(uint32_t, FunctionalState) {}
void RCC_APB2PeriphClockCmd(uint32_t, FunctionalState) {}
void GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*) {}
void GPIO_PinAFConfig(GPIO_TypeDef*, uint16_t, uint8_t) {}
void GPIO_ResetBits(GPIO_TypeDef*, uint16_t) {}
void GPIO_SetBits(GPIO_TypeDef*, uint16_t) {}
void GPIO_StructInit(GPIO_InitTypeDef*) {}
void gpioInit(GPIO_TypeDef*, gpio_config_t*) {}
void ADC_Cmd(ADC_TypeDef*, FunctionalState) {}
void ADC_CommonInit(ADC_TypeDef*, ADC_CommonInitTypeDef*) {}
void ADC_CommonStructInit(ADC_CommonInitTypeDef*) {}
void ADC_DMACmd(ADC_TypeDef*, FunctionalState) {}
void ADC_DMAConfig(ADC_TypeDef*, uint32_t) {}
FlagStatus ADC_GetFlagStatus(ADC_TypeDef*, uint32_t) { return SET; }
void ADC_Init(ADC_TypeDef*, ADC_InitTypeDef*) {}
void ADC_RegularChannelConfig(ADC_TypeDef*, uint8_t, uint8_t, uint8_t) {}
void ADC_StartConversion(ADC_TypeDef*) {}
void ADC_StructInit(ADC_InitTypeDef*) {}
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState) {}
void DMA_DeInit(DMA_Channel_TypeDef*) {}
void DMA_Init(DMA_Channel_TypeDef*, DMA_InitTypeDef*) {}
void DMA_StructInit(DMA_InitTypeDef*) {}
void SPI_Cmd(SPI_TypeDef*, FunctionalState) {}
void SPI_I2S_DeInit(SPI_TypeDef*) {}
void SPI_Init(SPI_TypeDef*, SPI_InitTypeDef*) {}
void SPI_RxFIFOThresholdConfig(SPI_TypeDef*, uint16_t) {}
bool spiInit(SPI_TypeDef*) { return false; }
bool spiTransfer(SPI_TypeDef*, uint8_t*, const uint8_t*, int) { return false; }
uint8_t spiTransferByte(SPI_TypeDef*, uint8_t) { return 0; }

// --- I2C bus and OLED -------------------------------------------------------
uint8_t i2cRead(uint8_t, uint8_t, uint8_t, uint8_t*) { return false; }
bool i2cWrite(uint8_t, uint8_t, uint8_t) { return false; }
bool i2cWriteBuffer(uint8_t, uint8_t, uint8_t, uint8_t*) { return false; }
uint16_t i2cGetErrorCounter(void) { return 0; }
void i2c_OLED_set_xy(uint8_t, uint8_t) {}
void i2c_OLED_send_string(const char*) {}
void i2c_OLED_clear_display_quick(void) {}
void i2c_OLED_send_changed_bytes(uint8_t*, uint8_t*, int) {}
bool ug2864hsweg01InitI2C(void) { return false; }
// Full character range (the real table covers printable ASCII from 32);
// all-zero glyphs.
const uint8_t multiWiiFont[256][5] = {};

// --- M25P16 blackbox flash: never ready ---------------------------------------
namespace {
const flashGeometry_t kNoFlash = {};
}  // namespace
const flashGeometry_t* m25p16_getGeometry() { return &kNoFlash; }
int m25p16_readBytes(uint32_t, uint8_t*, int) { return 0; }
bool m25p16_isReady() { return false; }
void m25p16_eraseSector(uint32_t) {}
void m25p16_eraseCompletely() {}
void m25p16_pageProgramBegin(uint32_t) {}
void m25p16_pageProgramContinue(const uint8_t*, int) {}
void m25p16_pageProgramFinish() {}

// --- VL53L0X ranging ------------------------------------------------------------
void LaserSensor::init() {}
void LaserSensor::setAddress(uint8_t) {}
int16_t LaserSensor::startRanging() { return 0; }

// --- optical flow globals (OPTIC_FLOW disabled on PRIMUSX2) ---------------------
float bodyRate[2] = {0.0f, 0.0f};
float flowRate[2] = {0.0f, 0.0f};
uint32_t last_opticflow_update_ms = 0;
uint16_t NewSensorRange = 0;

// --- PPM / PWM receivers (not used: FEATURE_RX_MSP) ----------------------------
uint16_t ppmRead(uint8_t) { return 0; }
uint16_t pwmRead(uint8_t) { return 0; }
bool isPPMDataBeingReceived(void) { return false; }
bool isPWMDataBeingReceived(void) { return false; }
void resetPPMDataReceivedState(void) {}

// --- UART and LEDs ------------------------------------------------------------
serialPort_t* uartOpen(USART_TypeDef*, serialReceiveCallbackPtr, uint32_t,
                       portMode_t, portOptions_t) {
  return nullptr;
}
void ledOperator(uint32_t, uint32_t) {}
