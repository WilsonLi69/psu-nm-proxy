#include "config.hpp"
#include "inventory.hpp"
#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "poller.hpp"

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

void logConfig(const psu::config::Config& config)
{
    std::cerr << "psu-nm-proxy: config psuNames=";
    for (const auto& [slot, name] : config.psuNames)
    {
        std::cerr << slot << ":" << name << " ";
    }
    std::cerr << "expectedSlots=";
    for (const auto slot : config.expectedSlots)
    {
        std::cerr << static_cast<int>(slot) << " ";
    }
    std::cerr << "psuAddresses=";
    for (const auto& [slot, address] : config.psuAddresses)
    {
        std::cerr << static_cast<int>(slot) << ":0x" << std::hex
                  << static_cast<int>(address) << std::dec << " ";
    }
    std::cerr << "chassisId=" << config.chassisId << " ";
    std::cerr << "d7hFallbackEnabled=" << config.d7hFallbackEnabled << " ";
    std::cerr << "sensorWhitelist=";
    for (const auto& label : config.sensorWhitelist)
    {
        std::cerr << label << " ";
    }
    std::cerr << "pollIntervalMs=" << config.pollIntervalMs
              << " failuresBeforeNotFunctional="
              << config.failuresBeforeNotFunctional << " thresholds=";
    for (const auto& [label, _] : config.thresholds)
    {
        std::cerr << label << " ";
    }
    std::cerr << "\n";
}

// entity-manager may not have created the Board/Chassis object yet when the
// daemon starts, so retry the discovery the association target depends on.
constexpr auto chassisDiscoveryRetry = std::chrono::seconds(2);

// State carried between the asynchronous startup steps. Shared via shared_ptr
// because the callback chain outlives the main function's call frames.
struct Discovery
{
    boost::asio::io_context& io;
    sdbusplus::asio::connection& conn;
    sdbusplus::asio::object_server& server;
    const psu::config::Config& config;
    std::chrono::milliseconds pollInterval;
    boost::asio::steady_timer chassisTimer;

    std::vector<std::unique_ptr<psu::sensors::Poller>> pollers;
    std::vector<psu::inventory::PsuInventory> inventory;
    std::vector<std::pair<std::string, std::uint8_t>> slots;
    std::size_t assetsPending = 0;

    Discovery(boost::asio::io_context& io, sdbusplus::asio::connection& conn,
              sdbusplus::asio::object_server& server,
              const psu::config::Config& config,
              std::chrono::milliseconds pollInterval) :
        io(io), conn(conn), server(server), config(config),
        pollInterval(pollInterval), chassisTimer(io)
    {}
};

void startPsuConfig(const std::shared_ptr<Discovery>& d);

// Attach the powered_by associations to the discovered Board/Chassis object,
// retrying until entity-manager publishes it (spec §5.6).
void attachChassisAssociation(const std::shared_ptr<Discovery>& d,
                              std::vector<std::string> psuPaths)
{
    psu::inventory::findBoardOrChassisPathsAsync(
        d->conn,
        [d, psuPaths = std::move(psuPaths)](
            std::optional<std::vector<std::string>> candidates) mutable {
            std::string chassisPath;
            if (candidates)
            {
                chassisPath = psu::inventory::selectChassisPath(
                    *candidates, d->config.chassisId);
            }
            else
            {
                std::cerr << "psu-nm-proxy: Board/Chassis discovery failed\n";
            }

            if (!chassisPath.empty())
            {
                try
                {
                    psu::inventory::publishChassisAssociation(d->server,
                                                              psuPaths,
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

            if (d->config.chassisId.empty())
            {
                std::cerr << "psu-nm-proxy: chassisId not configured; not "
                             "publishing PSU associations\n";
                return;
            }

            std::cerr << "psu-nm-proxy: no Board/Chassis object for "
                         "chassisId '"
                      << d->config.chassisId << "' yet; retrying\n";
            d->chassisTimer.expires_after(chassisDiscoveryRetry);
            d->chassisTimer.async_wait(
                [d, psuPaths](const boost::system::error_code& ec) mutable {
                    if (!ec)
                    {
                        attachChassisAssociation(d, std::move(psuPaths));
                    }
                });
        });
}

// Publish the inventory, associations and pollers once discovery is complete.
void finishDiscovery(const std::shared_ptr<Discovery>& d)
{
    try
    {
        const auto handles = psu::inventory::publish(d->server, d->inventory);
        const auto sensorDefs = psu::config::selectSensors(d->config);

        // bmcweb builds the Redfish PowerSupplies collection by resolving the
        // chassis id (the object leaf) against a Board/Chassis object, then
        // following that object's powered_by association. Attach the
        // association to that discovered object, never to a hard-coded path
        // (spec §5.6).
        std::vector<std::string> psuPaths;
        psuPaths.reserve(d->inventory.size());
        for (const auto& psu : d->inventory)
        {
            psuPaths.push_back(psu::inventory::powerSupplyPath(psu.name));
        }
        attachChassisAssociation(d, std::move(psuPaths));

        for (const auto& [name, address8] : d->slots)
        {
            const auto slotHandles = handles.at(name);
            auto poller = std::make_unique<psu::sensors::Poller>(
                d->io, d->conn, d->server, name, address8, d->pollInterval,
                d->config.failuresBeforeNotFunctional, sensorDefs,
                d->config.thresholds,
                [slotHandles](bool present, bool functional) {
                    psu::inventory::setPresentAndFunctional(slotHandles,
                                                            present,
                                                            functional);
                });
            poller->start();
            d->pollers.push_back(std::move(poller));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "psu-nm-proxy: D8h slot discovery failed: " << e.what()
                  << "\n";
    }
}

// Read one slot's Asset asynchronously; `index` addresses the pre-sized
// inventory vector. Only slots with a usable asset are polled (spec §5.6).
void readSlotAsset(const std::shared_ptr<Discovery>& d, std::size_t index,
                   const std::string& name, std::uint8_t address8)
{
    psu::inventory::readAssetAsync(
        d->conn, address8,
        [d, index, name, address8](
            std::optional<psu::inventory::PsuAsset> asset) {
            if (asset)
            {
                d->inventory[index] =
                    psu::inventory::makeInventory(name, asset);
                d->slots.emplace_back(name, address8);
            }

            if (--d->assetsPending == 0)
            {
                finishDiscovery(d);
            }
        });
}

// Slots are resolved; kick off the per-slot asset reads.
void onPsuConfigResolved(const std::shared_ptr<Discovery>& d,
                         const std::vector<std::uint8_t>& psuConfig)
{
    const auto resolved =
        psu::config::resolveSlots(d->config, psu::nm::psuSlots(psuConfig));

    d->inventory.clear();
    d->slots.clear();

    // Pre-size so each asset callback can fill its slot by index.
    d->inventory.reserve(resolved.size());
    for (const auto& slot : resolved)
    {
        d->inventory.push_back(
            psu::inventory::makeInventory(slot.name, std::nullopt));
    }

    // Count the addressable slots before starting any read: an Ipmb call can
    // complete synchronously on an immediate error, so a callback must never
    // see the pending count reach zero before every read has been started.
    d->assetsPending = 0;
    for (const auto& slot : resolved)
    {
        if (slot.address8)
        {
            ++d->assetsPending;
        }
    }
    if (d->assetsPending == 0)
    {
        finishDiscovery(d);
        return;
    }

    for (std::size_t i = 0; i < resolved.size(); ++i)
    {
        const auto& slot = resolved[i];
        // An expected slot D8h did not report has no address: publish it
        // absent and never poll it (spec §5.6).
        if (slot.address8)
        {
            readSlotAsset(d, i, slot.name, *slot.address8);
        }
    }
}

// D8h has been read; apply the D7h fallback once if configured and needed
// (spec §5.3), otherwise proceed to slot resolution.
void onPsuConfigRead(const std::shared_ptr<Discovery>& d,
                     psu::ipmb::Response psuConfig)
{
    if (!psuConfig)
    {
        std::cerr << "psu-nm-proxy: D8h slot discovery failed\n";
        return;
    }

    auto fallback = psu::nm::setPsuConfigFallback(
        d->config.d7hFallbackEnabled, *psuConfig, 0,
        psu::config::psuAddressBytes(d->config));
    if (!fallback)
    {
        onPsuConfigResolved(d, *psuConfig);
        return;
    }

    psu::ipmb::sendCommandAsync(
        d->conn, psu::nm::setPsuConfigCmd, *fallback,
        [d](psu::ipmb::Response /*written*/) {
            psu::ipmb::sendCommandAsync(
                d->conn, psu::nm::getPsuConfigCmd,
                psu::nm::encodeGetPsuConfig(),
                [d](psu::ipmb::Response reread) {
                    if (!reread)
                    {
                        std::cerr << "psu-nm-proxy: D8h slot discovery "
                                     "failed\n";
                        return;
                    }
                    onPsuConfigResolved(d, *reread);
                });
        });
}

void startPsuConfig(const std::shared_ptr<Discovery>& d)
{
    psu::ipmb::sendCommandAsync(d->conn, psu::nm::getPsuConfigCmd,
                                psu::nm::encodeGetPsuConfig(),
                                [d](psu::ipmb::Response psuConfig) {
                                    onPsuConfigRead(d, std::move(psuConfig));
                                });
}

// The startup gate: the ME must advertise the PMBUS proxy as supported (`01b`)
// or supported and enabled (`11b`) before we publish (spec §5.2, §5.3).
void onCapabilities(const std::shared_ptr<Discovery>& d,
                    psu::ipmb::Response caps)
{
    bool publishEnabled = false;
    if (caps)
    {
        publishEnabled = psu::nm::pmbusProxyEnabled(*caps);
    }
    else
    {
        std::cerr << "psu-nm-proxy: DEh capabilities read failed\n";
    }

    if (!publishEnabled)
    {
        std::cerr << "psu-nm-proxy: Intel ME PMBUS proxy not enabled; "
                     "publishing disabled\n";
        return;
    }

    startPsuConfig(d);
}

} // namespace

int main(int argc, char* argv[])
{
    const std::string configPath =
        argc > 1 ? argv[1] : std::string(psu::config::defaultConfigPath);
    const auto config = psu::config::loadConfig(configPath);
    logConfig(config);

    const auto pollInterval = std::chrono::milliseconds(config.pollIntervalMs);

    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.psu-nm-proxy");

    sdbusplus::asio::object_server server(conn);

    // Kick off the startup discovery asynchronously so the D-Bus service stays
    // answerable from the moment it is on the bus: no synchronous ME/IPMB read
    // ever blocks the io_context thread.
    auto discovery = std::make_shared<Discovery>(io, *conn, server, config,
                                                 pollInterval);
    psu::ipmb::sendCommandAsync(
        *conn, psu::nm::getFwCapabilitiesCmd,
        psu::nm::encodeGetFwCapabilities(),
        [discovery](psu::ipmb::Response caps) {
            onCapabilities(discovery, std::move(caps));
        });

    // Keep the process alive even when the proxy is disabled and no poller
    // exists, and run the io_context that serves D-Bus.
    auto keepAlive = boost::asio::make_work_guard(io);
    io.run();

    return 0;
}
