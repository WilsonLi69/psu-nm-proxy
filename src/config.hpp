#pragma once

#include "nm_command.hpp"
#include "sensor_set.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace psu::config
{

// Declarative daemon config; installed by the meta-custom recipe (spec §6).
inline constexpr std::string_view defaultConfigPath =
    "/etc/psu-nm-proxy/config.json";
inline constexpr std::uint32_t defaultPollIntervalMs = 1000;
inline constexpr std::size_t defaultFailuresBeforeNotFunctional = 3;

// All §8 labels that are polled and published, in sensor-set order.
std::vector<std::string> defaultSensorWhitelist();

// Resolved configuration. Omitted fields keep their defaults: psuNames is empty
// (each slot falls back to "PSU<slot>"), expectedSlots is empty (all discovered
// slots), sensorWhitelist is the full §8 set, pollIntervalMs 1000,
// failuresBeforeNotFunctional 3, and thresholds empty (no D-Bus threshold
// properties published).
struct Config
{
    std::map<std::uint8_t, std::string> psuNames;
    std::vector<std::uint8_t> expectedSlots;
    std::vector<std::string> sensorWhitelist = defaultSensorWhitelist();
    std::uint32_t pollIntervalMs = defaultPollIntervalMs;
    std::size_t failuresBeforeNotFunctional = defaultFailuresBeforeNotFunctional;
    std::map<std::string, psu::sensors::Thresholds> thresholds;

    bool operator==(const Config&) const = default;
};

// Parse config JSON. Invalid JSON, wrong field types, and out-of-range values
// are logged and fall back to defaults; this never throws.
Config parseConfig(std::string_view json);

// Read and parse a config file. A missing or unreadable file logs and yields
// defaults.
Config loadConfig(const std::string& path);

// The published name for a slot: the configured psuNames entry, or "PSU<slot>".
std::string nameFor(const Config& config, std::uint8_t slot);

// One configured PSU slot, matched against discovery.
struct ResolvedSlot
{
    std::uint8_t number;
    std::uint8_t address8;
    std::string name;

    bool operator==(const ResolvedSlot&) const = default;
};

// Match discovered D8h slots against the config: when expectedSlots is
// non-empty only those slots are kept, otherwise every discovered slot; the
// name comes from nameFor (spec §6).
std::vector<ResolvedSlot>
    resolveSlots(const Config& config,
                 const std::vector<psu::nm::PsuSlot>& discovered);

// The SensorDefs selected by sensorWhitelist, in whitelist order. Unknown
// labels are ignored.
std::vector<const psu::sensors::SensorDef*> selectSensors(const Config& config);

} // namespace psu::config
