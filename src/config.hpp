#pragma once

#include "nm_command.hpp"
#include "sensor_set.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
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
// slots), chassisId is empty (the sole Board/Chassis object is used when there
// is exactly one), sensorWhitelist is the full §8 set, pollIntervalMs 1000,
// failuresBeforeNotFunctional 3, and thresholds empty (no D-Bus threshold
// properties published).
struct Config
{
    std::map<std::uint8_t, std::string> psuNames;
    std::vector<std::uint8_t> expectedSlots;
    std::map<std::uint8_t, std::uint8_t> psuAddresses;
    std::string chassisId;
    bool d7hFallbackEnabled = false;
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

// The D7h fallback PSU addresses as slots 1..8 mapped to array indices 0..7
// (a slot with no configured address is 0).
std::array<std::uint8_t, 8> psuAddressBytes(const Config& config);

// One PSU slot to publish, from the union of the configured expectedSlots and
// the slots D8h discovered. An expected slot D8h did not report falls back to
// its configured psuAddresses entry when present (spec §5.6).
struct ResolvedSlot
{
    std::uint8_t number;
    std::optional<std::uint8_t> address8;
    std::string name;

    bool operator==(const ResolvedSlot&) const = default;
};

// Union the expected slots with the D8h-discovered slots: every expected slot
// is represented (expected slots first, in config order), then any discovered
// slot not in expectedSlots. The name comes from nameFor (spec §6).
std::vector<ResolvedSlot>
    resolveSlots(const Config& config,
                 const std::vector<psu::nm::PsuSlot>& discovered);

// The SensorDefs selected by sensorWhitelist, in whitelist order. Unknown
// labels are ignored.
std::vector<const psu::sensors::SensorDef*> selectSensors(const Config& config);

} // namespace psu::config
