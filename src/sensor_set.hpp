#pragma once

#include <array>
#include <cstdint>
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

// D-Bus path segment for a unit class (/xyz/openbmc_project/sensors/<class>).
inline constexpr std::string_view unitClassName(UnitClass unit)
{
    switch (unit)
    {
        case UnitClass::voltage:
            return "voltage";
        case UnitClass::current:
            return "current";
        case UnitClass::power:
            return "power";
        case UnitClass::temperature:
            return "temperature";
        case UnitClass::fan:
            return "fan";
    }
    return "voltage";
}

// xyz.openbmc_project.Sensor.Value.Unit enum string for a unit class.
inline std::string unitType(UnitClass unit)
{
    switch (unit)
    {
        case UnitClass::voltage:
            return "xyz.openbmc_project.Sensor.Value.Unit.Volts";
        case UnitClass::current:
            return "xyz.openbmc_project.Sensor.Value.Unit.Amperes";
        case UnitClass::power:
            return "xyz.openbmc_project.Sensor.Value.Unit.Watts";
        case UnitClass::temperature:
            return "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
        case UnitClass::fan:
            return "xyz.openbmc_project.Sensor.Value.Unit.RPMS";
    }
    return {};
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

} // namespace psu::sensors
