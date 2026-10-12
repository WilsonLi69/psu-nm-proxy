#include "sensor_set.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> labels()
{
    std::vector<std::string> result;
    for (const auto& def : psu::sensors::sensorSet)
    {
        result.emplace_back(def.label);
    }
    std::sort(result.begin(), result.end());
    return result;
}

const psu::sensors::SensorDef& defFor(std::string_view label)
{
    const auto* def = psu::sensors::findSensor(label);
    EXPECT_NE(def, nullptr);
    return *def;
}

} // namespace

// The §8 unit-typed set: vin, iin, pin, vout, iout, pout, temp1/2/3, fan1.
TEST(SensorSet, ContainsExactlyTheSection8UnitLabels)
{
    const std::vector<std::string> expected = {
        "fan1", "iin",  "iout", "pin", "pout",
        "temp1", "temp2", "temp3", "vin", "vout",
    };
    EXPECT_EQ(labels(), expected);
}

TEST(SensorSet, MapsEachLabelToItsUnitClass)
{
    EXPECT_EQ(defFor("vin").unit, psu::sensors::UnitClass::voltage);
    EXPECT_EQ(defFor("vout").unit, psu::sensors::UnitClass::voltage);
    EXPECT_EQ(defFor("iin").unit, psu::sensors::UnitClass::current);
    EXPECT_EQ(defFor("iout").unit, psu::sensors::UnitClass::current);
    EXPECT_EQ(defFor("pin").unit, psu::sensors::UnitClass::power);
    EXPECT_EQ(defFor("pout").unit, psu::sensors::UnitClass::power);
    EXPECT_EQ(defFor("temp1").unit, psu::sensors::UnitClass::temperature);
    EXPECT_EQ(defFor("temp2").unit, psu::sensors::UnitClass::temperature);
    EXPECT_EQ(defFor("temp3").unit, psu::sensors::UnitClass::temperature);
    EXPECT_EQ(defFor("fan1").unit, psu::sensors::UnitClass::fan);
}

TEST(SensorSet, MapsEachLabelToItsPmbusCommand)
{
    EXPECT_EQ(defFor("vin").command, 0x88);
    EXPECT_EQ(defFor("iin").command, 0x89);
    EXPECT_EQ(defFor("pin").command, 0x97);
    EXPECT_EQ(defFor("vout").command, 0x8B);
    EXPECT_EQ(defFor("iout").command, 0x8C);
    EXPECT_EQ(defFor("pout").command, 0x96);
    EXPECT_EQ(defFor("temp1").command, 0x8D);
    EXPECT_EQ(defFor("temp2").command, 0x8E);
    EXPECT_EQ(defFor("temp3").command, 0x8F);
    EXPECT_EQ(defFor("fan1").command, 0x90);
}

TEST(SensorSet, OnlyVoutUsesLinear16)
{
    EXPECT_EQ(defFor("vout").format, psu::sensors::Format::linear16);
    for (const auto& def : psu::sensors::sensorSet)
    {
        if (def.label != "vout")
        {
            EXPECT_EQ(def.format, psu::sensors::Format::linear11)
                << def.label;
        }
    }
}

TEST(SensorSet, ExcludesFanSpeed2)
{
    for (const auto& def : psu::sensors::sensorSet)
    {
        EXPECT_NE(def.command, 0x91) << def.label;
    }
}

TEST(SensorSet, NonUnitTypedCommandsAreDefined)
{
    EXPECT_EQ(psu::sensors::statusWordCommand, 0x79);
    EXPECT_EQ(psu::sensors::voutModeCommand, 0x20);
    EXPECT_EQ(psu::sensors::mfrIdCommand, 0x99);
    EXPECT_EQ(psu::sensors::mfrModelCommand, 0x9A);
}

TEST(SensorSet, UnitTypeStrings)
{
    EXPECT_EQ(psu::sensors::unitType(psu::sensors::UnitClass::voltage),
              "xyz.openbmc_project.Sensor.Value.Unit.Volts");
    EXPECT_EQ(psu::sensors::unitType(psu::sensors::UnitClass::current),
              "xyz.openbmc_project.Sensor.Value.Unit.Amperes");
    EXPECT_EQ(psu::sensors::unitType(psu::sensors::UnitClass::power),
              "xyz.openbmc_project.Sensor.Value.Unit.Watts");
    EXPECT_EQ(psu::sensors::unitType(psu::sensors::UnitClass::temperature),
              "xyz.openbmc_project.Sensor.Value.Unit.DegreesC");
    EXPECT_EQ(psu::sensors::unitType(psu::sensors::UnitClass::fan),
              "xyz.openbmc_project.Sensor.Value.Unit.RPMS");
}

TEST(SensorSet, BuildsUnitTypedSensorPath)
{
    EXPECT_EQ(psu::sensors::sensorPath("PSU2", defFor("vin")),
              "/xyz/openbmc_project/sensors/voltage/PSU2_VIN");
    EXPECT_EQ(psu::sensors::sensorPath("PSU2", defFor("vout")),
              "/xyz/openbmc_project/sensors/voltage/PSU2_VOUT");
    EXPECT_EQ(psu::sensors::sensorPath("PSU1", defFor("iin")),
              "/xyz/openbmc_project/sensors/current/PSU1_IIN");
    EXPECT_EQ(psu::sensors::sensorPath("PSU2", defFor("pout")),
              "/xyz/openbmc_project/sensors/power/PSU2_POUT");
    EXPECT_EQ(psu::sensors::sensorPath("PSU2", defFor("temp1")),
              "/xyz/openbmc_project/sensors/temperature/PSU2_TEMP1");
    EXPECT_EQ(psu::sensors::sensorPath("PSU2", defFor("fan1")),
              "/xyz/openbmc_project/sensors/fan/PSU2_FAN1");
}

TEST(SensorSet, UnknownLabelIsNotFound)
{
    EXPECT_EQ(psu::sensors::findSensor("nope"), nullptr);
}
