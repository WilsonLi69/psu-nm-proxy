#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace psu::sensors
{

// Unit-typed Sensor.Value classes published for a PSU (spec §5.6).
enum class UnitClass
{
    voltage,
    current,
    power,
    temperature,
    fan,
};

// Decode format used by a sensor's PMBus command.
enum class Format
{
    linear11,
    linear16,
};

// One published sensor: its D-Bus label, unit class, PMBus read command and
// decode format.
struct SensorDef
{
    std::string_view label;
    UnitClass unit;
    std::uint8_t command;
    Format format;
};

// The §8 unit-typed sensor set (spec §5.6/§8). READ_FAN_SPEED_2 (0x91) and
// any separate 5 VSB rail are deliberately excluded (unsupported through the
// ME proxy; see docs/research/10-acbel-psu-command-set.md).
inline constexpr std::array<SensorDef, 10> sensorSet = {{
    {"vin", UnitClass::voltage, 0x88, Format::linear11},
    {"iin", UnitClass::current, 0x89, Format::linear11},
    {"pin", UnitClass::power, 0x97, Format::linear11},
    {"vout", UnitClass::voltage, 0x8B, Format::linear16},
    {"iout", UnitClass::current, 0x8C, Format::linear11},
    {"pout", UnitClass::power, 0x96, Format::linear11},
    {"temp1", UnitClass::temperature, 0x8D, Format::linear11},
    {"temp2", UnitClass::temperature, 0x8E, Format::linear11},
    {"temp3", UnitClass::temperature, 0x8F, Format::linear11},
    {"fan1", UnitClass::fan, 0x90, Format::linear11},
}};

// §8 commands that are not unit-typed Sensor.Value objects: STATUS_WORD drives
// OperationalStatus, VOUT_MODE supplies the Linear16 exponent, and the MFR_*
// strings feed the inventory Asset (#18).
inline constexpr std::uint8_t statusWordCommand = 0x79;
inline constexpr std::uint8_t voutModeCommand = 0x20;
inline constexpr std::uint8_t mfrIdCommand = 0x99;
inline constexpr std::uint8_t mfrModelCommand = 0x9A;

// D-Bus attributes of a unit class: the sensors/<class> path segment and the
// xyz.openbmc_project.Sensor.Value.Unit enum string.
struct UnitInfo
{
    std::string_view className;
    std::string_view unitType;
};

struct UnitEntry
{
    UnitClass unit;
    UnitInfo info;
};

inline constexpr std::array<UnitEntry, 5> unitTable = {{
    {UnitClass::voltage,
     {"voltage", "xyz.openbmc_project.Sensor.Value.Unit.Volts"}},
    {UnitClass::current,
     {"current", "xyz.openbmc_project.Sensor.Value.Unit.Amperes"}},
    {UnitClass::power,
     {"power", "xyz.openbmc_project.Sensor.Value.Unit.Watts"}},
    {UnitClass::temperature,
     {"temperature", "xyz.openbmc_project.Sensor.Value.Unit.DegreesC"}},
    {UnitClass::fan,
     {"fan_tach", "xyz.openbmc_project.Sensor.Value.Unit.RPMS"}},
}};

inline UnitInfo unitInfo(UnitClass unit)
{
    for (const auto& entry : unitTable)
    {
        if (entry.unit == unit)
        {
            return entry.info;
        }
    }
    return unitTable.front().info;
}

// D-Bus path segment for a unit class (/xyz/openbmc_project/sensors/<class>).
inline std::string_view unitClassName(UnitClass unit)
{
    return unitInfo(unit).className;
}

// xyz.openbmc_project.Sensor.Value.Unit enum string for a unit class.
inline std::string unitType(UnitClass unit)
{
    return std::string(unitInfo(unit).unitType);
}

// Published object path: /xyz/openbmc_project/sensors/<class>/<PSU>_<LABEL>.
inline std::string sensorPath(std::string_view psuName, const SensorDef& def)
{
    std::string label(def.label);
    for (char& c : label)
    {
        if (c >= 'a' && c <= 'z')
        {
            c = static_cast<char>(c - 'a' + 'A');
        }
    }

    return "/xyz/openbmc_project/sensors/" +
           std::string(unitClassName(def.unit)) + "/" +
           std::string(psuName) + "_" + label;
}

// Per-label Sensor.Threshold bounds (spec §6). Each bound is optional: only the
// metrics present in config are published as D-Bus properties.
struct Thresholds
{
    std::optional<double> upperWarning;
    std::optional<double> lowerWarning;
    std::optional<double> upperCritical;
    std::optional<double> lowerCritical;

    bool operator==(const Thresholds&) const = default;
};

// D-Bus threshold interfaces (spec §5.6).
inline constexpr auto warningThresholdInterface =
    "xyz.openbmc_project.Sensor.Threshold.Warning";
inline constexpr auto criticalThresholdInterface =
    "xyz.openbmc_project.Sensor.Threshold.Critical";

// Property maps for the threshold interfaces. An empty map means that
// interface must not be created (a label with no configured thresholds
// publishes no threshold properties).
struct ThresholdProperties
{
    std::map<std::string, double> warning;
    std::map<std::string, double> critical;

    bool empty() const
    {
        return warning.empty() && critical.empty();
    }

    bool operator==(const ThresholdProperties&) const = default;
};

// Map configured bounds onto their D-Bus property names: WarningHigh /
// WarningLow on the Warning interface, CriticalHigh / CriticalLow on the
// Critical interface (spec §5.6).
inline ThresholdProperties thresholdProperties(const Thresholds& thresholds)
{
    ThresholdProperties props;
    if (thresholds.upperWarning)
    {
        props.warning["WarningHigh"] = *thresholds.upperWarning;
    }
    if (thresholds.lowerWarning)
    {
        props.warning["WarningLow"] = *thresholds.lowerWarning;
    }
    if (thresholds.upperCritical)
    {
        props.critical["CriticalHigh"] = *thresholds.upperCritical;
    }
    if (thresholds.lowerCritical)
    {
        props.critical["CriticalLow"] = *thresholds.lowerCritical;
    }
    return props;
}

// Look up a sensor definition by its §8 label, or nullptr if unknown.
inline const SensorDef* findSensor(std::string_view label)
{
    for (const auto& def : sensorSet)
    {
        if (def.label == label)
        {
            return &def;
        }
    }
    return nullptr;
}

// Initial D-Bus property values for a published sensor object at startup
// (spec §5.6): populated slots start at Value=0.0, Functional=true,
// Available=true, State=Enabled; unpopulated slots start at Value=NaN,
// Functional=false, Available=false, State=Disabled.
struct InitialSensorState
{
    double value;
    bool functional;
    bool available;
    std::string_view state;
};

inline constexpr InitialSensorState initialSensorState(bool initiallyPresent)
{
    if (initiallyPresent)
    {
        return {0.0, true, true,
                "xyz.openbmc_project.State.Decorator.OperationalStatus.State.Enabled"};
    }
    return {std::numeric_limits<double>::quiet_NaN(), false, false,
            "xyz.openbmc_project.State.Decorator.OperationalStatus.State.Disabled"};
}

} // namespace psu::sensors
