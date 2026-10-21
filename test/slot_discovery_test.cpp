#include "slot_discovery.hpp"

#include <gtest/gtest.h>

#include <boost/asio/io_context.hpp>

#include <optional>
#include <string>
#include <vector>

namespace
{

class FakeNodeManagerClient : public psu::discovery::NodeManagerClient
{
  public:
    void getCapabilities(ResponseCallback callback) override
    {
        capabilitiesCalls++;
        callback(capabilitiesResponse);
    }

    void getPsuConfig(ResponseCallback callback) override
    {
        psuConfigCalls++;
        callback(psuConfigResponse);
    }

    void setPsuConfig(const std::vector<std::uint8_t>& payload,
                      ResponseCallback callback) override
    {
        setPsuConfigPayloads.push_back(payload);
        if (onSetPsuConfig)
        {
            onSetPsuConfig();
        }
        callback(setPsuConfigResponse);
    }

    void readAsset(std::uint8_t psuAddr8,
                   std::function<void(std::optional<psu::inventory::PsuAsset>)>
                       callback) override
    {
        assetCalls.push_back(psuAddr8);
        const auto it = assetResponses.find(psuAddr8);
        callback(it != assetResponses.end() ? it->second : std::nullopt);
    }

    int capabilitiesCalls = 0;
    Response capabilitiesResponse;

    int psuConfigCalls = 0;
    Response psuConfigResponse;

    std::vector<std::vector<std::uint8_t>> setPsuConfigPayloads;
    Response setPsuConfigResponse;
    std::function<void()> onSetPsuConfig;

    std::vector<std::uint8_t> assetCalls;
    std::map<std::uint8_t, std::optional<psu::inventory::PsuAsset>> assetResponses;
};

class FakeObjectMapperClient : public psu::discovery::ObjectMapperClient
{
  public:
    void findBoardOrChassisPaths(PathsCallback callback) override
    {
        findCalls++;
        callback(pathsResponse);
    }

    int findCalls = 0;
    std::optional<std::vector<std::string>> pathsResponse;
};

class FakeInventorySink : public psu::discovery::InventorySink
{
  public:
    std::map<std::string, HealthSink> publishInventory(
        const std::vector<psu::inventory::PsuInventory>& inventory) override
    {
        publishedInventory.push_back(inventory);
        std::map<std::string, HealthSink> result;
        for (const auto& item : inventory)
        {
            result[item.name] = [this, name = item.name](bool present, bool functional) {
                healthUpdates[name].emplace_back(present, functional);
            };
        }
        return result;
    }

    void publishChassisAssociation(
        const std::vector<std::string>& psuPaths, std::string_view chassisPath,
        const std::vector<std::string>& sensorPaths = {}) override
    {
        publishedChassisAssociations.emplace_back(psuPaths,
                                                  std::string(chassisPath));
        publishedSensorAssociations.push_back(sensorPaths);
    }

    std::vector<std::vector<psu::inventory::PsuInventory>> publishedInventory;
    std::vector<std::pair<std::vector<std::string>, std::string>>
        publishedChassisAssociations;
    std::vector<std::vector<std::string>> publishedSensorAssociations;
    std::map<std::string, std::vector<std::pair<bool, bool>>> healthUpdates;
};

std::vector<std::uint8_t> validCapabilities()
{
    std::vector<std::uint8_t> caps(9, 0);
    caps[8] = 0x04; // proxy = 01b (supported)
    return caps;
}

} // namespace

TEST(SlotDiscoveryTest, ReportsProxyDisabledWhenCapabilitiesFail)
{
    boost::asio::io_context io;
    psu::config::Config config;
    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = std::nullopt; // Failed DEh read

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::proxyDisabled);
    EXPECT_EQ(nm.capabilitiesCalls, 1);
    EXPECT_EQ(nm.psuConfigCalls, 0);
}

TEST(SlotDiscoveryTest, ReportsTransportErrorWhenD8hFails)
{
    boost::asio::io_context io;
    psu::config::Config config;
    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = validCapabilities();
    nm.psuConfigResponse = std::nullopt; // D8h read failed

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::transportError);
    EXPECT_EQ(nm.capabilitiesCalls, 1);
    EXPECT_EQ(nm.psuConfigCalls, 1);
}

TEST(SlotDiscoveryTest, AppliesD7hFallbackWhenConfiguredAndSlotsEmpty)
{
    boost::asio::io_context io;
    psu::config::Config config;
    config.d7hFallbackEnabled = true;
    config.psuAddresses[2] = 0xB2;

    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = validCapabilities();
    // First D8h response: 9 zeroes (slot 0 domain, slots 1-8 empty)
    std::vector<std::uint8_t> emptyD8(9, 0);
    // After fallback written, second D8h response returns slot 2 with address 0xB2
    std::vector<std::uint8_t> configuredD8(9, 0);
    configuredD8[2] = 0xB2;

    nm.psuConfigResponse = emptyD8;
    nm.setPsuConfigResponse = std::vector<std::uint8_t>{};
    nm.onSetPsuConfig = [&nm, configuredD8]() {
        nm.psuConfigResponse = configuredD8;
    };
    nm.assetResponses[0xB2] = psu::inventory::PsuAsset{"ACBEL", "R1BA2751A"};

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    EXPECT_EQ(nm.psuConfigCalls, 2);
    ASSERT_EQ(nm.setPsuConfigPayloads.size(), 1U);
    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::success);
    ASSERT_EQ(outcome->slots.size(), 1U);
    EXPECT_EQ(outcome->slots[0].name, "PSU2");
    EXPECT_EQ(outcome->slots[0].address8, 0xB2);
}

TEST(SlotDiscoveryTest, YieldsTransportErrorWhenD7hWriteFails)
{
    boost::asio::io_context io;
    psu::config::Config config;
    config.d7hFallbackEnabled = true;
    config.psuAddresses[2] = 0xB2;

    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = validCapabilities();
    std::vector<std::uint8_t> emptyD8(9, 0);
    nm.psuConfigResponse = emptyD8;
    nm.setPsuConfigResponse = std::nullopt; // D7h write fails

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::transportError);
    EXPECT_EQ(nm.psuConfigCalls, 1);
    EXPECT_EQ(nm.setPsuConfigPayloads.size(), 1U);
}

TEST(SlotDiscoveryTest, ResolvesSlotsReadsAssetsAndPublishesInventory)
{
    boost::asio::io_context io;
    psu::config::Config config;
    config.expectedSlots = {1, 2};
    config.psuAddresses = {{1, 0xB0}, {2, 0xB2}};
    config.chassisId = "chassis";

    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = validCapabilities();
    // D8h returns Slot 2 configured at 0xB2 (Slot 1 falls back to psuAddresses)
    std::vector<std::uint8_t> d8(9, 0);
    d8[2] = 0xB2;
    nm.psuConfigResponse = d8;

    // Slot 1 asset times out (absent at boot), Slot 2 asset responds (present)
    nm.assetResponses[0xB0] = std::nullopt;
    nm.assetResponses[0xB2] = psu::inventory::PsuAsset{"ACBEL", "R1BA2751A-P00A"};

    // Chassis path
    mapper.pathsResponse = std::vector<std::string>{
        "/xyz/openbmc_project/inventory/system/chassis",
    };

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::success);
    ASSERT_EQ(outcome->slots.size(), 2U);
    EXPECT_EQ(outcome->slots[0].name, "PSU1");
    EXPECT_EQ(outcome->slots[0].address8, 0xB0);
    EXPECT_FALSE(outcome->slots[0].present);
    EXPECT_EQ(outcome->slots[1].name, "PSU2");
    EXPECT_EQ(outcome->slots[1].address8, 0xB2);
    EXPECT_TRUE(outcome->slots[1].present);

    // Verify published inventory: 2 items (PSU1 absent, PSU2 present)
    ASSERT_EQ(sink.publishedInventory.size(), 1U);
    const auto& inv = sink.publishedInventory[0];
    ASSERT_EQ(inv.size(), 2U);
    EXPECT_EQ(inv[0].name, "PSU1");
    EXPECT_FALSE(inv[0].present);
    EXPECT_FALSE(inv[0].functional);
    EXPECT_EQ(inv[1].name, "PSU2");
    EXPECT_TRUE(inv[1].present);
    EXPECT_TRUE(inv[1].functional);
    EXPECT_EQ(inv[1].manufacturer, "ACBEL");

    // Verify health callback linkage for both unpopulated and populated slots
    outcome->slots[0].updateInventoryHealth(true, true);
    ASSERT_EQ(sink.healthUpdates["PSU1"].size(), 1U);
    EXPECT_EQ(sink.healthUpdates["PSU1"][0], (std::pair<bool, bool>{true, true}));

    outcome->slots[1].updateInventoryHealth(true, false);
    ASSERT_EQ(sink.healthUpdates["PSU2"].size(), 1U);
    EXPECT_EQ(sink.healthUpdates["PSU2"][0], (std::pair<bool, bool>{true, false}));

    // Verify chassis association published (both PSU paths and all 20 sensor paths)
    ASSERT_EQ(sink.publishedChassisAssociations.size(), 1U);
    EXPECT_EQ(sink.publishedChassisAssociations[0].second,
              "/xyz/openbmc_project/inventory/system/chassis");
    ASSERT_EQ(sink.publishedSensorAssociations.size(), 1U);
    EXPECT_EQ(sink.publishedSensorAssociations[0].size(), 20U);
}

TEST(SlotDiscoveryTest, ReturnsSlotWithPresentFalseWhenAssetReadFails)
{
    boost::asio::io_context io;
    psu::config::Config config;
    config.expectedSlots = {1};

    FakeNodeManagerClient nm;
    FakeObjectMapperClient mapper;
    FakeInventorySink sink;

    nm.capabilitiesResponse = validCapabilities();
    // D8h returns Slot 1 configured at 0xB0
    std::vector<std::uint8_t> d8(9, 0);
    d8[1] = 0xB0;
    nm.psuConfigResponse = d8;

    // Asset read fails (e.g. empty slot)
    nm.assetResponses[0xB0] = std::nullopt;

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, nm, mapper, sink);

    std::optional<psu::discovery::DiscoveryOutcome> outcome;
    discovery->start([&outcome](psu::discovery::DiscoveryOutcome res) {
        outcome = std::move(res);
    });

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status,
              psu::discovery::DiscoveryOutcome::Status::success);
    // Unpopulated slot is still returned for poller & sensor provisioning with present = false
    ASSERT_EQ(outcome->slots.size(), 1U);
    EXPECT_EQ(outcome->slots[0].name, "PSU1");
    EXPECT_EQ(outcome->slots[0].address8, 0xB0);
    EXPECT_FALSE(outcome->slots[0].present);

    // Published inventory has PSU1 marked absent and not functional
    ASSERT_EQ(sink.publishedInventory.size(), 1U);
    ASSERT_EQ(sink.publishedInventory[0].size(), 1U);
    EXPECT_FALSE(sink.publishedInventory[0][0].present);
    EXPECT_FALSE(sink.publishedInventory[0][0].functional);
}


