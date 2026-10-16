#include "slot_discovery.hpp"

#include "nm_command.hpp"

#include <exception>
#include <iostream>
#include <utility>

namespace psu::discovery
{

namespace
{
constexpr auto chassisDiscoveryRetry = std::chrono::seconds(2);
}

SlotDiscovery::SlotDiscovery(boost::asio::io_context& io,
                             const psu::config::Config& config,
                             NodeManagerClient& nmClient,
                             ObjectMapperClient& mapperClient,
                             InventorySink& inventorySink) :
    io(io), config(config), nmClient(nmClient), mapperClient(mapperClient),
    inventorySink(inventorySink), chassisTimer(io)
{}

void SlotDiscovery::start(DiscoveryCallback callback)
{
    this->callback = std::move(callback);
    nmClient.getCapabilities(
        [self = shared_from_this()](NodeManagerClient::Response caps) {
            self->onCapabilities(std::move(caps));
        });
}

void SlotDiscovery::onCapabilities(NodeManagerClient::Response caps)
{
    bool publishEnabled = false;
    if (caps)
    {
        publishEnabled = psu::nm::pmbusProxyEnabled(*caps);
    }

    if (!publishEnabled)
    {
        if (callback)
        {
            auto cb = std::move(callback);
            cb(DiscoveryOutcome{
                DiscoveryOutcome::Status::proxyDisabled,
                "Intel ME PMBUS proxy not enabled; publishing disabled",
                {}});
        }
        return;
    }

    readPsuConfig();
}

void SlotDiscovery::readPsuConfig()
{
    nmClient.getPsuConfig(
        [self = shared_from_this()](NodeManagerClient::Response psuConfig) {
            self->onPsuConfigRead(std::move(psuConfig));
        });
}

void SlotDiscovery::onPsuConfigRead(NodeManagerClient::Response psuConfig)
{
    if (!psuConfig)
    {
        if (callback)
        {
            auto cb = std::move(callback);
            cb(DiscoveryOutcome{DiscoveryOutcome::Status::transportError,
                                "D8h slot discovery failed",
                                {}});
        }
        return;
    }

    auto fallback = psu::nm::setPsuConfigFallback(
        config.d7hFallbackEnabled, *psuConfig, 0,
        psu::config::psuAddressBytes(config));
    if (!fallback)
    {
        onPsuConfigResolved(*psuConfig);
        return;
    }

    nmClient.setPsuConfig(
        *fallback,
        [self = shared_from_this()](NodeManagerClient::Response written) {
            if (!written)
            {
                if (self->callback)
                {
                    auto cb = std::move(self->callback);
                    cb(DiscoveryOutcome{
                        DiscoveryOutcome::Status::transportError,
                        "D7h set PSU config failed",
                        {}});
                }
                return;
            }
            self->nmClient.getPsuConfig(
                [self](NodeManagerClient::Response reread) {
                    if (!reread)
                    {
                        if (self->callback)
                        {
                            auto cb = std::move(self->callback);
                            cb(DiscoveryOutcome{
                                DiscoveryOutcome::Status::transportError,
                                "D8h slot discovery failed",
                                {}});
                        }
                        return;
                    }
                    self->onPsuConfigResolved(*reread);
                });
        });
}

void SlotDiscovery::onPsuConfigResolved(
    const std::vector<std::uint8_t>& psuConfig)
{
    const auto resolved =
        psu::config::resolveSlots(config, psu::nm::psuSlots(psuConfig));

    inventory.clear();
    activeSlots.clear();

    inventory.reserve(resolved.size());
    for (const auto& slot : resolved)
    {
        inventory.push_back(
            psu::inventory::makeInventory(slot.name, std::nullopt));
    }

    assetsPending = 0;
    for (const auto& slot : resolved)
    {
        if (slot.address8)
        {
            ++assetsPending;
        }
    }
    if (assetsPending == 0)
    {
        finishDiscovery();
        return;
    }

    for (std::size_t i = 0; i < resolved.size(); ++i)
    {
        const auto& slot = resolved[i];
        if (slot.address8)
        {
            readSlotAsset(i, slot.name, *slot.address8);
        }
    }
}

void SlotDiscovery::readSlotAsset(std::size_t index, const std::string& name,
                                  std::uint8_t address8)
{
    nmClient.readAsset(
        address8,
        [self = shared_from_this(), index, name, address8](
            std::optional<psu::inventory::PsuAsset> asset) {
            if (asset)
            {
                self->inventory[index] =
                    psu::inventory::makeInventory(name, asset);
                self->activeSlots.emplace_back(name, address8);
            }

            if (--self->assetsPending == 0)
            {
                self->finishDiscovery();
            }
        });
}

void SlotDiscovery::finishDiscovery()
{
    try
    {
        auto healthMap = inventorySink.publishInventory(inventory);

        std::vector<std::string> psuPaths;
        psuPaths.reserve(inventory.size());
        for (const auto& psu : inventory)
        {
            psuPaths.push_back(psu::inventory::powerSupplyPath(psu.name));
        }
        attachChassisAssociation(std::move(psuPaths));

        std::vector<DiscoveredSlot> readySlots;
        readySlots.reserve(activeSlots.size());
        for (const auto& [name, address8] : activeSlots)
        {
            auto it = healthMap.find(name);
            InventorySink::HealthSink updater = [](bool, bool) {};
            if (it != healthMap.end())
            {
                updater = it->second;
            }
            readySlots.push_back(
                DiscoveredSlot{name, address8, std::move(updater)});
        }

        if (callback)
        {
            auto cb = std::move(callback);
            cb(DiscoveryOutcome{DiscoveryOutcome::Status::success,
                                "Discovery successful",
                                std::move(readySlots)});
        }
    }
    catch (const std::exception& e)
    {
        if (callback)
        {
            auto cb = std::move(callback);
            cb(DiscoveryOutcome{DiscoveryOutcome::Status::transportError,
                                std::string("Discovery failed: ") + e.what(),
                                {}});
        }
    }
}

void SlotDiscovery::attachChassisAssociation(std::vector<std::string> psuPaths)
{
    mapperClient.findBoardOrChassisPaths(
        [self = shared_from_this(), psuPaths = std::move(psuPaths)](
            std::optional<std::vector<std::string>> candidates) mutable {
            std::string chassisPath;
            if (candidates)
            {
                chassisPath = psu::inventory::selectChassisPath(
                    *candidates, self->config.chassisId);
            }
            else
            {
                std::cerr << "psu-nm-proxy: Board/Chassis discovery failed\n";
            }

            if (!chassisPath.empty())
            {
                try
                {
                    self->inventorySink.publishChassisAssociation(psuPaths,
                                                                  chassisPath);
                    std::cerr << "psu-nm-proxy: powered_by associations on "
                              << chassisPath << "\n";
                }
                catch (const std::exception& e)
                {
                    std::cerr << "psu-nm-proxy: publishing PSU associations "
                                 "failed: "
                              << e.what() << "\n";
                }
                return;
            }

            if (self->config.chassisId.empty())
            {
                std::cerr << "psu-nm-proxy: chassisId not configured; not "
                             "publishing PSU associations\n";
                return;
            }

            std::cerr << "psu-nm-proxy: no Board/Chassis object for chassisId '"
                      << self->config.chassisId << "' yet; retrying\n";
            self->chassisTimer.expires_after(chassisDiscoveryRetry);
            self->chassisTimer.async_wait(
                [self, psuPaths = std::move(psuPaths)](
                    const boost::system::error_code& ec) mutable {
                    if (!ec)
                    {
                        self->attachChassisAssociation(std::move(psuPaths));
                    }
                });
        });
}

} // namespace psu::discovery
