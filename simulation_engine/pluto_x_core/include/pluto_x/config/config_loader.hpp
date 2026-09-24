// Copyright 2026 AVATIC contributors.
//
// YAML loading and validation of LegacyStackConfig.
//
// Every parameter is required: there are no silent defaults. A missing,
// non-numeric, non-finite or out-of-range value raises ConfigError with the
// full YAML key path, e.g. "controller.rate.roll.kp".

#ifndef PLUTO_X_CONFIG_CONFIG_LOADER_HPP_
#define PLUTO_X_CONFIG_CONFIG_LOADER_HPP_

#include <optional>
#include <stdexcept>
#include <string>

#include "pluto_x/config/legacy_params.hpp"

namespace pluto_x {

class ConfigError : public std::runtime_error {
 public:
  explicit ConfigError(const std::string& message)
      : std::runtime_error(message) {}
};

/// Loads and validates a configuration file. Throws ConfigError.
/// flight_controller_type, if given ("legacy" or "magisv2"), replaces
/// flight_controller.type before validation (launch-time selection).
LegacyStackConfig LoadLegacyStackConfigFile(
    const std::string& path,
    const std::optional<std::string>& flight_controller_type = std::nullopt);

/// Loads and validates a configuration from YAML text. Throws ConfigError.
LegacyStackConfig LoadLegacyStackConfigString(const std::string& yaml_text);

/// Validates cross-field constraints of an already populated configuration.
/// Throws ConfigError. Called by the loaders; exposed for tests and for code
/// that constructs configurations programmatically.
void ValidateLegacyStackConfig(const LegacyStackConfig& config);

/// Human-readable multi-line summary for initialization logging.
std::string DescribeLegacyStackConfig(const LegacyStackConfig& config);

}  // namespace pluto_x

#endif  // PLUTO_X_CONFIG_CONFIG_LOADER_HPP_
