#include "config.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace
{

using psu::config::Config;
using psu::config::ResolvedSlot;
using psu::nm::PsuSlot;

std::vector<std::string> sorted(std::vector<std::string> values)
{
    std::sort(values.begin(), values.end());
    return values;
}

std::vector<std::string> allSensorLabels()
{
    std::vector<std::string> labels;
    for (const auto& def : psu::sensors::sensorSet)
    {
        labels.emplace_back(def.label);
    }
    return sorted(std::move(labels));
}

} // namespace

TEST(Config, DefaultPathConstant)
{
    EXPECT_EQ(psu::config::defaultConfigPath, "/etc/psu-nm-proxy/config.json");
}

TEST(ConfigDefaults, EmptyObjectUsesDefaults)
{
    const auto cfg = psu::config::parseConfig("{}");

    EXPECT_TRUE(cfg.psuNames.empty());
    EXPECT_TRUE(cfg.expectedSlots.empty());
    EXPECT_EQ(sorted(cfg.sensorWhitelist), allSensorLabels());
    EXPECT_EQ(cfg.pollIntervalMs, 1000U);
    EXPECT_EQ(cfg.failuresBeforeNotFunctional, 3U);
    EXPECT_EQ(cfg, Config{});
}

TEST(ConfigParsing, FullConfig)
{
    const auto cfg = psu::config::parseConfig(R"({
        "psuNames": {"1": "PSU1", "2": "PSU2"},
        "expectedSlots": [1, 2],
        "sensorWhitelist": ["vin", "vout"],
        "pollIntervalMs": 500,
        "failuresBeforeNotFunctional": 5
    })");

    ASSERT_EQ(cfg.psuNames.size(), 2U);
    EXPECT_EQ(cfg.psuNames.at(1), "PSU1");
    EXPECT_EQ(cfg.psuNames.at(2), "PSU2");
    EXPECT_EQ(cfg.expectedSlots, (std::vector<std::uint8_t>{1, 2}));
    EXPECT_EQ(cfg.sensorWhitelist, (std::vector<std::string>{"vin", "vout"}));
    EXPECT_EQ(cfg.pollIntervalMs, 500U);
    EXPECT_EQ(cfg.failuresBeforeNotFunctional, 5U);
}

TEST(ConfigParsing, PartialConfigKeepsOtherDefaults)
{
    const auto cfg = psu::config::parseConfig(R"({"pollIntervalMs": 2000})");

    EXPECT_EQ(cfg.pollIntervalMs, 2000U);
    EXPECT_EQ(cfg.failuresBeforeNotFunctional, 3U);
    EXPECT_TRUE(cfg.psuNames.empty());
    EXPECT_TRUE(cfg.expectedSlots.empty());
    EXPECT_EQ(sorted(cfg.sensorWhitelist), allSensorLabels());
}

TEST(ConfigParsing, InvalidJsonFallsBackToDefaults)
{
    EXPECT_EQ(psu::config::parseConfig("{ this is not json"), Config{});
    EXPECT_EQ(psu::config::parseConfig(""), Config{});
}

TEST(ConfigParsing, WrongFieldTypesFallBackToDefaults)
{
    const auto cfg = psu::config::parseConfig(R"({
        "psuNames": 5,
        "expectedSlots": "two",
        "sensorWhitelist": 7,
        "pollIntervalMs": "soon",
        "failuresBeforeNotFunctional": false
    })");

    EXPECT_EQ(cfg, Config{});
}

TEST(ConfigParsing, OutOfRangeAndNegativeValuesUseDefaults)
{
    const auto cfg = psu::config::parseConfig(R"({
        "psuNames": {"0": "zero", "9": "nine", "2": ""},
        "expectedSlots": [0, 9, -1],
        "sensorWhitelist": ["vin", "bogus"],
        "pollIntervalMs": 0,
        "failuresBeforeNotFunctional": -2
    })");

    EXPECT_TRUE(cfg.psuNames.empty());
    EXPECT_TRUE(cfg.expectedSlots.empty());
    EXPECT_EQ(cfg.sensorWhitelist, (std::vector<std::string>{"vin"}));
    EXPECT_EQ(cfg.pollIntervalMs, 1000U);
    EXPECT_EQ(cfg.failuresBeforeNotFunctional, 3U);
}

TEST(ConfigParsing, PsuNamesKeepsValidEntriesAndSkipsInvalid)
{
    const auto cfg = psu::config::parseConfig(
        R"({"psuNames": {"1": "Left", "3": 4, "2": "Right"}})");

    ASSERT_EQ(cfg.psuNames.size(), 2U);
    EXPECT_EQ(cfg.psuNames.at(1), "Left");
    EXPECT_EQ(cfg.psuNames.at(2), "Right");
}

TEST(ConfigParsing, ExpectedSlotsDedupesAndPreservesOrder)
{
    const auto cfg = psu::config::parseConfig(R"({"expectedSlots": [2, 1, 2]})");

    EXPECT_EQ(cfg.expectedSlots, (std::vector<std::uint8_t>{2, 1}));
}

TEST(ConfigParsing, PsuAddressesAcceptNumbersAndHexStrings)
{
    const auto cfg = psu::config::parseConfig(R"({
        "psuAddresses": {"1": 176, "2": "0xB2", "3": "179"}
    })");

    ASSERT_EQ(cfg.psuAddresses.size(), 3U);
    EXPECT_EQ(cfg.psuAddresses.at(1), 0xB0);
    EXPECT_EQ(cfg.psuAddresses.at(2), 0xB2);
    EXPECT_EQ(cfg.psuAddresses.at(3), 0xB3);
}

TEST(ConfigParsing, PsuAddressesIgnoreInvalidEntries)
{
    const auto cfg = psu::config::parseConfig(R"({
        "psuAddresses": {
            "1": "0x100", "2": "nope", "3": 256, "4": -1,
            "5": true, "0": "0xB2", "9": "0xB2", "6": "0xB2"
        }
    })");

    ASSERT_EQ(cfg.psuAddresses.size(), 1U);
    EXPECT_EQ(cfg.psuAddresses.at(6), 0xB2);
}

TEST(ConfigParsing, PsuAddressesWrongTypeIsIgnored)
{
    EXPECT_TRUE(psu::config::parseConfig(R"({"psuAddresses": 5})")
                    .psuAddresses.empty());
    EXPECT_TRUE(psu::config::parseConfig(R"({"psuAddresses": ["0xB2"]})")
                    .psuAddresses.empty());
}

TEST(ConfigParsing, D7hFallbackDefaultsOff)
{
    EXPECT_FALSE(psu::config::parseConfig("{}").d7hFallbackEnabled);
}

TEST(ConfigParsing, D7hFallbackEnabledFromConfig)
{
    EXPECT_TRUE(psu::config::parseConfig(R"({"d7hFallbackEnabled": true})")
                    .d7hFallbackEnabled);
    EXPECT_FALSE(psu::config::parseConfig(R"({"d7hFallbackEnabled": false})")
                     .d7hFallbackEnabled);
}

TEST(ConfigParsing, D7hFallbackWrongTypeUsesDefault)
{
    EXPECT_FALSE(psu::config::parseConfig(R"({"d7hFallbackEnabled": "yes"})")
                     .d7hFallbackEnabled);
}

TEST(ConfigParsing, ChassisIdDefaultsToEmpty)
{
    EXPECT_TRUE(psu::config::parseConfig("{}").chassisId.empty());
}

TEST(ConfigParsing, ChassisIdFromConfig)
{
    EXPECT_EQ(psu::config::parseConfig(R"({"chassisId": "custom_board"})")
                  .chassisId,
              "custom_board");
}

TEST(ConfigParsing, ChassisIdWrongTypeOrUnsafeFallsBackToEmpty)
{
    EXPECT_TRUE(
        psu::config::parseConfig(R"({"chassisId": 5})").chassisId.empty());
    EXPECT_TRUE(
        psu::config::parseConfig(R"({"chassisId": ""})").chassisId.empty());
    EXPECT_TRUE(psu::config::parseConfig(R"({"chassisId": "a/b"})")
                    .chassisId.empty());
}

TEST(ConfigResolution, PsuAddressBytesIndexBySlot)
{
    Config cfg;
    cfg.psuAddresses = {{2, 0xB2}, {1, 0xB0}};

    const std::array<std::uint8_t, 8> expected = {0xB0, 0xB2, 0, 0,
                                                  0,    0,    0, 0};
    EXPECT_EQ(psu::config::psuAddressBytes(cfg), expected);
}

TEST(ConfigResolution, NameFallsBackToPsuSlot)
{
    const Config cfg;

    EXPECT_EQ(psu::config::nameFor(cfg, 2), "PSU2");
}

TEST(ConfigResolution, NameUsesConfiguredOverride)
{
    Config cfg;
    cfg.psuNames[3] = "Bay3";

    EXPECT_EQ(psu::config::nameFor(cfg, 3), "Bay3");
}

TEST(ConfigResolution, ResolvesEveryDiscoveredSlotByDefault)
{
    const Config cfg;
    const std::vector<PsuSlot> discovered = {{1, 0xB0}, {2, 0xB2}};

    const auto slots = psu::config::resolveSlots(cfg, discovered);

    ASSERT_EQ(slots.size(), 2U);
    EXPECT_EQ(slots[0], (ResolvedSlot{1, 0xB0, "PSU1"}));
    EXPECT_EQ(slots[1], (ResolvedSlot{2, 0xB2, "PSU2"}));
}

TEST(ConfigResolution, ExpectedSlotsAndDiscoveryAreUnioned)
{
    Config cfg;
    cfg.expectedSlots = {2};
    cfg.psuNames[2] = "Right";
    const std::vector<PsuSlot> discovered = {{1, 0xB0}, {2, 0xB2}};

    const auto slots = psu::config::resolveSlots(cfg, discovered);

    ASSERT_EQ(slots.size(), 2U);
    EXPECT_EQ(slots[0], (ResolvedSlot{2, 0xB2, "Right"}));
    EXPECT_EQ(slots[1], (ResolvedSlot{1, 0xB0, "PSU1"}));
}

// An expected slot that D8h does not report is still represented, with no
// address, so it is published but never polled (spec §5.6).
TEST(ConfigResolution, ExpectedSlotNotDiscoveredIsStillRepresented)
{
    Config cfg;
    cfg.expectedSlots = {1, 3};
    const std::vector<PsuSlot> discovered = {{2, 0xB2}};

    const auto slots = psu::config::resolveSlots(cfg, discovered);

    ASSERT_EQ(slots.size(), 3U);
    EXPECT_EQ(slots[0], (ResolvedSlot{1, std::nullopt, "PSU1"}));
    EXPECT_EQ(slots[1], (ResolvedSlot{3, std::nullopt, "PSU3"}));
    EXPECT_EQ(slots[2], (ResolvedSlot{2, 0xB2, "PSU2"}));
}

TEST(ConfigSensors, DefaultWhitelistSelectsEverySection8Label)
{
    const Config cfg;

    EXPECT_EQ(psu::config::selectSensors(cfg).size(),
              psu::sensors::sensorSet.size());
}

TEST(ConfigSensors, WhitelistSelectsAndOrdersKnownLabels)
{
    Config cfg;
    cfg.sensorWhitelist = {"vout", "vin", "bogus"};

    const auto sensors = psu::config::selectSensors(cfg);

    ASSERT_EQ(sensors.size(), 2U);
    EXPECT_EQ(sensors[0]->label, "vout");
    EXPECT_EQ(sensors[1]->label, "vin");
}

TEST(ConfigSensors, ExplicitEmptyWhitelistSelectsNothing)
{
    Config cfg;
    cfg.sensorWhitelist = {};

    EXPECT_TRUE(psu::config::selectSensors(cfg).empty());
}

TEST(ConfigLoad, ReadsAndParsesFile)
{
    const auto path = std::filesystem::temp_directory_path() /
                      "psu-nm-proxy-config-test.json";
    {
        std::ofstream out(path);
        out << R"({"pollIntervalMs": 250, "expectedSlots": [2]})";
    }

    const auto cfg = psu::config::loadConfig(path.string());

    EXPECT_EQ(cfg.pollIntervalMs, 250U);
    EXPECT_EQ(cfg.expectedSlots, (std::vector<std::uint8_t>{2}));

    std::filesystem::remove(path);
}

TEST(ConfigLoad, MissingFileFallsBackToDefaults)
{
    EXPECT_EQ(psu::config::loadConfig("/nonexistent/psu-nm-proxy/config.json"),
              Config{});
}
