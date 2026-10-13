#include "inventory.hpp"

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

TEST(Inventory, InterfaceAndPathConstants)
{
    EXPECT_EQ(psu::inventory::powerSupplyInterface,
              "xyz.openbmc_project.Inventory.Item.PowerSupply");
    EXPECT_EQ(psu::inventory::assetInterface,
              "xyz.openbmc_project.Inventory.Decorator.Asset");
    EXPECT_EQ(psu::inventory::itemInterface,
              "xyz.openbmc_project.Inventory.Item");
    EXPECT_EQ(psu::inventory::operationalStatusInterface,
              "xyz.openbmc_project.State.Decorator.OperationalStatus");
    EXPECT_EQ(psu::inventory::associationInterface,
              "xyz.openbmc_project.Association.Definitions");
    EXPECT_EQ(psu::inventory::boardInterface,
              "xyz.openbmc_project.Inventory.Item.Board");
    EXPECT_EQ(psu::inventory::chassisItemInterface,
              "xyz.openbmc_project.Inventory.Item.Chassis");
}

TEST(Inventory, SelectsChassisPathByMatchingFilename)
{
    const std::vector<std::string> candidates = {
        "/xyz/openbmc_project/inventory/system/chassis",
        "/xyz/openbmc_project/inventory/system/board/custom_board",
    };

    EXPECT_EQ(psu::inventory::selectChassisPath(candidates, "custom_board"),
              "/xyz/openbmc_project/inventory/system/board/custom_board");
}

TEST(Inventory, NoChassisPathWhenNoFilenameMatches)
{
    const std::vector<std::string> candidates = {
        "/xyz/openbmc_project/inventory/system/chassis",
    };

    EXPECT_TRUE(
        psu::inventory::selectChassisPath(candidates, "custom_board").empty());
}

TEST(Inventory, FallsBackToSoleCandidateWhenNoChassisId)
{
    const std::vector<std::string> candidates = {
        "/xyz/openbmc_project/inventory/system/board/custom_board",
    };

    EXPECT_EQ(psu::inventory::selectChassisPath(candidates, ""),
              candidates.front());
}

TEST(Inventory, NoChassisPathWhenNoIdAndMultipleCandidates)
{
    const std::vector<std::string> candidates = {
        "/xyz/openbmc_project/inventory/system/board/custom_board",
        "/xyz/openbmc_project/inventory/system/board/other",
    };

    EXPECT_TRUE(psu::inventory::selectChassisPath(candidates, "").empty());
}

TEST(Inventory, NoChassisPathWhenNoCandidates)
{
    EXPECT_TRUE(psu::inventory::selectChassisPath({}, "").empty());
    EXPECT_TRUE(psu::inventory::selectChassisPath({}, "custom_board").empty());
}

TEST(Inventory, BuildsPowerSupplyPath)
{
    EXPECT_EQ(psu::inventory::powerSupplyPath("PSU1"),
              "/xyz/openbmc_project/inventory/system/power_supply/PSU1");
    EXPECT_EQ(psu::inventory::powerSupplyPath("PSU2"),
              "/xyz/openbmc_project/inventory/system/power_supply/PSU2");
}

TEST(Inventory, PresentSlotCarriesAssetAndFunctional)
{
    const auto inv = psu::inventory::makeInventory(
        "PSU2", psu::inventory::PsuAsset{"ACBEL", "R1BA2751A-P00A"});

    EXPECT_EQ(inv.name, "PSU2");
    EXPECT_TRUE(inv.present);
    EXPECT_TRUE(inv.functional);
    EXPECT_EQ(inv.manufacturer, "ACBEL");
    EXPECT_EQ(inv.model, "R1BA2751A-P00A");
}

TEST(Inventory, EmptySlotIsAbsentAndNotFunctional)
{
    const auto inv = psu::inventory::makeInventory("PSU1", std::nullopt);

    EXPECT_EQ(inv.name, "PSU1");
    EXPECT_FALSE(inv.present);
    EXPECT_FALSE(inv.functional);
    EXPECT_EQ(inv.manufacturer, "");
    EXPECT_EQ(inv.model, "");
}

TEST(Inventory, AssetPropertiesCarryManufacturerAndModel)
{
    const auto inv = psu::inventory::makeInventory(
        "PSU2", psu::inventory::PsuAsset{"ACBEL", "R1BA2751A-P00A"});

    const std::map<std::string, std::string> expected = {
        {"Manufacturer", "ACBEL"},
        {"Model", "R1BA2751A-P00A"},
    };
    EXPECT_EQ(psu::inventory::assetProperties(inv), expected);
}

TEST(Inventory, AbsentSlotAssetPropertiesAreEmptyStrings)
{
    const auto inv = psu::inventory::makeInventory("PSU1", std::nullopt);

    const std::map<std::string, std::string> expected = {
        {"Manufacturer", ""},
        {"Model", ""},
    };
    EXPECT_EQ(psu::inventory::assetProperties(inv), expected);
}

TEST(Inventory, PoweringAssociationTuple)
{
    const std::string path =
        "/xyz/openbmc_project/inventory/system/power_supply/PSU2";
    EXPECT_EQ(psu::inventory::poweringAssociation(path),
              (std::tuple<std::string, std::string, std::string>{
                  "powered_by", "powering", path}));
}

TEST(Inventory, ChassisCarriesOnePoweringAssociationPerPsu)
{
    const std::vector<std::string> paths = {
        "/xyz/openbmc_project/inventory/system/power_supply/PSU1",
        "/xyz/openbmc_project/inventory/system/power_supply/PSU2",
    };

    const auto associations =
        psu::inventory::chassisPoweringAssociations(paths);

    ASSERT_EQ(associations.size(), 2U);
    EXPECT_EQ(
        associations[0],
        (psu::inventory::Association{"powered_by", "powering", paths[0]}));
    EXPECT_EQ(
        associations[1],
        (psu::inventory::Association{"powered_by", "powering", paths[1]}));
}

TEST(Inventory, NoPsuMeansNoAssociations)
{
    EXPECT_TRUE(psu::inventory::chassisPoweringAssociations({}).empty());
}

TEST(Inventory, UsableAssetStringIsPrintableNonEmptyAscii)
{
    EXPECT_TRUE(psu::inventory::usableAssetString("ACBEL"));
    EXPECT_TRUE(psu::inventory::usableAssetString("R1BA2751A-P00A"));
    EXPECT_FALSE(psu::inventory::usableAssetString(""));
    EXPECT_FALSE(psu::inventory::usableAssetString(std::string("\x05", 1)));
    EXPECT_FALSE(psu::inventory::usableAssetString("AC\x7F"));
}
