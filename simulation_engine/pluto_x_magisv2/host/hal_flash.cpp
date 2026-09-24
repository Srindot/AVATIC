// Copyright 2026 AVATIC contributors. SPDX-License-Identifier: GPL-3.0-or-later
//
// Replaces the STM32 flash used for the configuration page (StdPeriph
// FLASH_* functions). Together with patches/0001-host-config-flash-buffer.patch
// the production EEPROM code (resetConf -> writeEEPROM -> checksum ->
// readEEPROM) runs unchanged against this buffer.
//
// The firmware passes flash addresses as uint32_t. The patch sets the config
// base address to this buffer, whose 64-bit address is truncated to 32 bits
// in those calls; offsets are therefore recovered with 32-bit modular
// arithmetic, which is exact for any offset inside the page.

#include <cstdint>
#include <cstring>

#include "firmware_includes.hpp"

namespace {

// FLASH_TO_RESERVE_FOR_CONFIG on STM32F303 (config.cpp): 2 kB.
constexpr std::uint32_t kConfigPageBytes = 0x800;

std::uint32_t OffsetOf(uint32_t address);

}  // namespace

extern "C" {
alignas(4) uint8_t magis_host_config_flash[kConfigPageBytes];
}

namespace {

std::uint32_t OffsetOf(uint32_t address) {
  const auto base = static_cast<std::uint32_t>(
      reinterpret_cast<std::uintptr_t>(magis_host_config_flash));
  return address - base;  // modulo 2^32
}

}  // namespace

void FLASH_Unlock(void) {}
void FLASH_Lock(void) {}
void FLASH_ClearFlag(uint32_t /*flags*/) {}

FLASH_Status FLASH_ErasePage(uint32_t page_address) {
  const std::uint32_t offset = OffsetOf(page_address);
  if (offset >= kConfigPageBytes) {
    return FLASH_ERROR_PROGRAM;
  }
  std::memset(magis_host_config_flash + offset, 0xFF,
              kConfigPageBytes - offset);
  return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uint32_t address, uint32_t data) {
  const std::uint32_t offset = OffsetOf(address);
  if (offset > kConfigPageBytes - sizeof(data)) {
    return FLASH_ERROR_PROGRAM;
  }
  std::memcpy(magis_host_config_flash + offset, &data, sizeof(data));
  return FLASH_COMPLETE;
}
