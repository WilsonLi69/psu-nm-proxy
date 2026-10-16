#pragma once

#include "config.hpp"
#include "inventory_sink.hpp"
#include "node_manager_client.hpp"
#include "object_mapper_client.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace psu::discovery
{

struct DiscoveredSlot
{
    std::string name;
    std::uint8_t address8;
    std::function<void(bool present, bool functional)> updateInventoryHealth;
};

struct DiscoveryOutcome
{
    enum class Status
    {
        success,
        proxyDisabled,
        transportError,
    };

    Status status;
    std::string message;
    std::vector<DiscoveredSlot> slots;
};

using DiscoveryCallback = std::function<void(DiscoveryOutcome)>;

class SlotDiscovery : public std::enable_shared_from_this<SlotDiscovery>
{
  public:
    SlotDiscovery(boost::asio::io_context& io,
                  const psu::config::Config& config,
                  NodeManagerClient& nmClient,
                  ObjectMapperClient& mapperClient,
                  InventorySink& inventorySink);

    void start(DiscoveryCallback callback);

  private:
    void onCapabilities(NodeManagerClient::Response caps);
    void readPsuConfig();
    void onPsuConfigRead(NodeManagerClient::Response psuConfig);
    void onPsuConfigResolved(const std::vector<std::uint8_t>& psuConfig);
    void readSlotAsset(std::size_t index, const std::string& name,
                       std::uint8_t address8);
    void finishDiscovery();
    void attachChassisAssociation(std::vector<std::string> psuPaths);

    boost::asio::io_context& io;
    const psu::config::Config& config;
    NodeManagerClient& nmClient;
    ObjectMapperClient& mapperClient;
    InventorySink& inventorySink;
    boost::asio::steady_timer chassisTimer;
    DiscoveryCallback callback;

    std::vector<psu::inventory::PsuInventory> inventory;
    std::vector<std::pair<std::string, std::uint8_t>> activeSlots;
    std::size_t assetsPending = 0;
};

} // namespace psu::discovery
