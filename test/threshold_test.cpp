#include "config.hpp"
#include "sensor_set.hpp"

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>

namespace
{

using psu::config::Config;
using psu::sensors::Thresholds;

Thresholds thresholds(double upperWarning, double lowerWarning,
                      double upperCritical, double lowerCritical)
{
    Thresholds t;
    t.upperWarning = upperWarning;
    t.lowerWarning = lowerWarning;
    t.upperCritical = upperCritical;
    t.lowerCritical = lowerCritical;
    return t;
}

} // namespace

TEST(ThresholdParse, AllFourFields)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {
            "vin": {"upperWarning": 200, "lowerWarning": 90,
                    "upperCritical": 220, "lowerCritical": 80}
        }
    })");

    ASSERT_EQ(cfg.thresholds.size(), 1U);
    EXPECT_EQ(cfg.thresholds.at("vin"),
              thresholds(200.0, 90.0, 220.0, 80.0));
}

TEST(ThresholdParse, PartialFields)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {"temp1": {"upperWarning": 60, "upperCritical": 90}}
    })");

    ASSERT_EQ(cfg.thresholds.size(), 1U);
    const auto& t = cfg.thresholds.at("temp1");
    EXPECT_EQ(t.upperWarning, std::optional<double>(60.0));
    EXPECT_FALSE(t.lowerWarning.has_value());
    EXPECT_EQ(t.upperCritical, std::optional<double>(90.0));
    EXPECT_FALSE(t.lowerCritical.has_value());
}

TEST(ThresholdParse, MapsPerLabelIndependently)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {
            "vin": {"upperCritical": 220},
            "temp1": {"lowerWarning": 5, "lowerCritical": 0}
        }
    })");

    ASSERT_EQ(cfg.thresholds.size(), 2U);
    EXPECT_EQ(cfg.thresholds.at("vin").upperCritical,
              std::optional<double>(220.0));
    EXPECT_FALSE(cfg.thresholds.at("vin").lowerWarning.has_value());
    EXPECT_EQ(cfg.thresholds.at("temp1").lowerWarning,
              std::optional<double>(5.0));
    EXPECT_EQ(cfg.thresholds.at("temp1").lowerCritical,
              std::optional<double>(0.0));
}

TEST(ThresholdParse, LabelWithoutThresholdsIsAbsent)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {"vin": {"upperWarning": 200}}
    })");

    EXPECT_EQ(cfg.thresholds.count("vin"), 1U);
    EXPECT_EQ(cfg.thresholds.count("temp1"), 0U);
}

TEST(ThresholdParse, MissingThresholdsObjectYieldsNone)
{
    EXPECT_TRUE(psu::config::parseConfig(R"({"pollIntervalMs": 500})")
                    .thresholds.empty());
    EXPECT_TRUE(
        psu::config::parseConfig(R"({"thresholds": {}})").thresholds.empty());
}

TEST(ThresholdParse, ThresholdsWrongTypeIsIgnored)
{
    EXPECT_TRUE(
        psu::config::parseConfig(R"({"thresholds": 5})").thresholds.empty());
    EXPECT_TRUE(psu::config::parseConfig(R"({"thresholds": [1, 2]})")
                    .thresholds.empty());
    EXPECT_TRUE(
        psu::config::parseConfig(R"({"thresholds": "vin"})").thresholds.empty());
}

TEST(ThresholdParse, MalformedValuesAreIgnored)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {
            "vin": {
                "upperWarning": "200",
                "lowerWarning": true,
                "upperCritical": null,
                "lowerCritical": {"nested": 1}
            }
        }
    })");

    EXPECT_TRUE(cfg.thresholds.empty());
}

TEST(ThresholdParse, NegativeValuesAreIgnored)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {"vin": {"upperWarning": -1, "lowerCritical": -5}}
    })");

    EXPECT_TRUE(cfg.thresholds.empty());
}

TEST(ThresholdParse, MixedValidAndInvalidKeepsOnlyValid)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {
            "vin": {"upperWarning": 200, "lowerWarning": -1,
                    "upperCritical": "high"}
        }
    })");

    ASSERT_EQ(cfg.thresholds.size(), 1U);
    const auto& t = cfg.thresholds.at("vin");
    EXPECT_EQ(t.upperWarning, std::optional<double>(200.0));
    EXPECT_FALSE(t.lowerWarning.has_value());
    EXPECT_FALSE(t.upperCritical.has_value());
}

TEST(ThresholdParse, UnknownLabelIsIgnored)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {"nope": {"upperWarning": 200}}
    })");

    EXPECT_TRUE(cfg.thresholds.empty());
}

TEST(ThresholdParse, LabelValueNotObjectIsIgnored)
{
    const auto cfg = psu::config::parseConfig(R"({
        "thresholds": {"vin": 200, "temp1": {"upperWarning": 60}}
    })");

    ASSERT_EQ(cfg.thresholds.size(), 1U);
    EXPECT_EQ(cfg.thresholds.count("vin"), 0U);
    EXPECT_EQ(cfg.thresholds.at("temp1").upperWarning,
              std::optional<double>(60.0));
}

TEST(ThresholdPublish, InterfaceNames)
{
    EXPECT_EQ(psu::sensors::warningThresholdInterface,
              "xyz.openbmc_project.Sensor.Threshold.Warning");
    EXPECT_EQ(psu::sensors::criticalThresholdInterface,
              "xyz.openbmc_project.Sensor.Threshold.Critical");
}

TEST(ThresholdPublish, MapsAllBoundsToPropertyNames)
{
    const auto props = psu::sensors::thresholdProperties(
        thresholds(200.0, 90.0, 220.0, 80.0));

    const std::map<std::string, double> warning = {
        {"WarningHigh", 200.0},
        {"WarningLow", 90.0},
    };
    const std::map<std::string, double> critical = {
        {"CriticalHigh", 220.0},
        {"CriticalLow", 80.0},
    };
    EXPECT_EQ(props.warning, warning);
    EXPECT_EQ(props.critical, critical);
    EXPECT_FALSE(props.empty());
}

TEST(ThresholdPublish, PartialBoundsPopulateOnlyPresentInterfaces)
{
    Thresholds t;
    t.upperWarning = 200.0;

    const auto props = psu::sensors::thresholdProperties(t);

    EXPECT_EQ(props.warning,
              (std::map<std::string, double>{{"WarningHigh", 200.0}}));
    EXPECT_TRUE(props.critical.empty());
    EXPECT_FALSE(props.empty());
}

TEST(ThresholdPublish, CriticalOnlyHasNoWarningInterface)
{
    Thresholds t;
    t.upperCritical = 90.0;
    t.lowerCritical = 10.0;

    const auto props = psu::sensors::thresholdProperties(t);

    EXPECT_TRUE(props.warning.empty());
    EXPECT_EQ(props.critical,
              (std::map<std::string, double>{{"CriticalHigh", 90.0},
                                             {"CriticalLow", 10.0}}));
}

TEST(ThresholdPublish, NoThresholdsYieldsNoProperties)
{
    const auto props = psu::sensors::thresholdProperties(Thresholds{});

    EXPECT_TRUE(props.warning.empty());
    EXPECT_TRUE(props.critical.empty());
    EXPECT_TRUE(props.empty());
}
