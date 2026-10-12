#include "config.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace psu::config
{

namespace
{

constexpr int minSlot = 1;
constexpr int maxSlot = 8;

void logFallback(const std::string& field)
{
    std::cerr << "psu-nm-proxy: config field '" << field
              << "' invalid; using default\n";
}

bool validSlot(int value)
{
    return value >= minSlot && value <= maxSlot;
}

std::optional<int> slotFromKey(std::string_view key)
{
    if (key.empty())
    {
        return std::nullopt;
    }
    int value = 0;
    const auto result =
        std::from_chars(key.data(), key.data() + key.size(), value);
    if (result.ec != std::errc{} || result.ptr != key.data() + key.size())
    {
        return std::nullopt;
    }
    return value;
}

void parsePsuNames(const nlohmann::json& value, Config& config)
{
    if (!value.is_object())
    {
        logFallback("psuNames");
        return;
    }
    for (const auto& [key, name] : value.items())
    {
        const auto slot = slotFromKey(key);
        if (!slot || !validSlot(*slot) || !name.is_string() ||
            name.get<std::string>().empty())
        {
            logFallback("psuNames");
            continue;
        }
        config.psuNames[static_cast<std::uint8_t>(*slot)] =
            name.get<std::string>();
    }
}

void parseExpectedSlots(const nlohmann::json& value, Config& config)
{
    if (!value.is_array())
    {
        logFallback("expectedSlots");
        return;
    }
    for (const auto& entry : value)
    {
        if (!entry.is_number_integer())
        {
            logFallback("expectedSlots");
            continue;
        }
        const int slot = entry.get<int>();
        if (!validSlot(slot))
        {
            logFallback("expectedSlots");
            continue;
        }
        const auto number = static_cast<std::uint8_t>(slot);
        if (std::find(config.expectedSlots.begin(), config.expectedSlots.end(),
                      number) == config.expectedSlots.end())
        {
            config.expectedSlots.push_back(number);
        }
    }
}

void parseSensorWhitelist(const nlohmann::json& value, Config& config)
{
    if (!value.is_array())
    {
        logFallback("sensorWhitelist");
        return;
    }
    std::vector<std::string> labels;
    for (const auto& entry : value)
    {
        if (!entry.is_string())
        {
            logFallback("sensorWhitelist");
            continue;
        }
        const auto label = entry.get<std::string>();
        if (psu::sensors::findSensor(label) == nullptr)
        {
            logFallback("sensorWhitelist");
            continue;
        }
        if (std::find(labels.begin(), labels.end(), label) == labels.end())
        {
            labels.push_back(label);
        }
    }
    config.sensorWhitelist = std::move(labels);
}

std::optional<std::int64_t> positiveInteger(const nlohmann::json& value)
{
    if (!value.is_number_integer())
    {
        return std::nullopt;
    }
    try
    {
        const auto number = value.get<std::int64_t>();
        if (number > 0)
        {
            return number;
        }
    }
    catch (const nlohmann::json::exception&)
    {
    }
    return std::nullopt;
}

void readThreshold(const nlohmann::json& entry, const char* key,
                   std::optional<double>& out)
{
    const auto it = entry.find(key);
    if (it == entry.end())
    {
        return;
    }
    if (!it->is_number())
    {
        logFallback("thresholds");
        return;
    }
    double value = 0.0;
    try
    {
        value = it->get<double>();
    }
    catch (const nlohmann::json::exception&)
    {
        logFallback("thresholds");
        return;
    }
    if (value < 0.0 || !std::isfinite(value))
    {
        logFallback("thresholds");
        return;
    }
    out = value;
}

void parseThresholds(const nlohmann::json& value, Config& config)
{
    if (!value.is_object())
    {
        logFallback("thresholds");
        return;
    }
    for (const auto& [label, entry] : value.items())
    {
        if (entry.is_object() &&
            psu::sensors::findSensor(label) != nullptr)
        {
            psu::sensors::Thresholds thresholds;
            readThreshold(entry, "upperWarning", thresholds.upperWarning);
            readThreshold(entry, "lowerWarning", thresholds.lowerWarning);
            readThreshold(entry, "upperCritical", thresholds.upperCritical);
            readThreshold(entry, "lowerCritical", thresholds.lowerCritical);
            if (thresholds.upperWarning || thresholds.lowerWarning ||
                thresholds.upperCritical || thresholds.lowerCritical)
            {
                config.thresholds[label] = thresholds;
                continue;
            }
        }
        logFallback("thresholds");
    }
}

} // namespace

std::vector<std::string> defaultSensorWhitelist()
{
    std::vector<std::string> labels;
    labels.reserve(psu::sensors::sensorSet.size());
    for (const auto& def : psu::sensors::sensorSet)
    {
        labels.emplace_back(def.label);
    }
    return labels;
}

Config parseConfig(std::string_view json)
{
    Config config;

    const auto doc = nlohmann::json::parse(json, nullptr, false);
    if (doc.is_discarded() || !doc.is_object())
    {
        if (!json.empty())
        {
            std::cerr << "psu-nm-proxy: config is not a JSON object; using "
                         "defaults\n";
        }
        return config;
    }

    if (const auto it = doc.find("psuNames"); it != doc.end())
    {
        parsePsuNames(*it, config);
    }

    if (const auto it = doc.find("expectedSlots"); it != doc.end())
    {
        parseExpectedSlots(*it, config);
    }

    if (const auto it = doc.find("sensorWhitelist"); it != doc.end())
    {
        parseSensorWhitelist(*it, config);
    }

    if (const auto it = doc.find("thresholds"); it != doc.end())
    {
        parseThresholds(*it, config);
    }

    if (const auto it = doc.find("pollIntervalMs"); it != doc.end())
    {
        if (const auto value = positiveInteger(*it);
            value && *value <= std::numeric_limits<std::uint32_t>::max())
        {
            config.pollIntervalMs = static_cast<std::uint32_t>(*value);
        }
        else
        {
            logFallback("pollIntervalMs");
        }
    }

    if (const auto it = doc.find("failuresBeforeNotFunctional");
        it != doc.end())
    {
        if (const auto value = positiveInteger(*it);
            value && *value <= std::numeric_limits<std::size_t>::max())
        {
            config.failuresBeforeNotFunctional = static_cast<std::size_t>(*value);
        }
        else
        {
            logFallback("failuresBeforeNotFunctional");
        }
    }

    return config;
}

Config loadConfig(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "psu-nm-proxy: cannot read config " << path
                  << "; using defaults\n";
        return {};
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parseConfig(buffer.str());
}

std::string nameFor(const Config& config, std::uint8_t slot)
{
    if (const auto it = config.psuNames.find(slot);
        it != config.psuNames.end())
    {
        return it->second;
    }
    return "PSU" + std::to_string(slot);
}

std::vector<ResolvedSlot>
    resolveSlots(const Config& config,
                 const std::vector<psu::nm::PsuSlot>& discovered)
{
    std::vector<ResolvedSlot> resolved;
    for (const auto& slot : discovered)
    {
        if (!config.expectedSlots.empty() &&
            std::find(config.expectedSlots.begin(), config.expectedSlots.end(),
                      slot.number) == config.expectedSlots.end())
        {
            continue;
        }
        resolved.push_back(
            {slot.number, slot.address8, nameFor(config, slot.number)});
    }
    return resolved;
}

std::vector<const psu::sensors::SensorDef*> selectSensors(const Config& config)
{
    std::vector<const psu::sensors::SensorDef*> sensors;
    for (const auto& label : config.sensorWhitelist)
    {
        if (const auto* def = psu::sensors::findSensor(label))
        {
            sensors.push_back(def);
        }
    }
    return sensors;
}

} // namespace psu::config
