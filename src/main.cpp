#include "inventory.hpp"
#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "poller.hpp"

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr auto pollInterval = std::chrono::seconds(1);

// Consecutive D9h failures before a slot is published not functional (spec
// §5.7). Ticket #20 will source this from the daemon config.
constexpr std::size_t failuresBeforeNotFunctional = 3;

// D7h is a fallback only and is never written by default (spec §5.3, §5.2).
constexpr bool enableSetPsuConfigFallback = false;
constexpr std::array<std::uint8_t, 8> configuredPsuAddresses8 = {};

std::vector<std::uint8_t> readPsuConfig(sdbusplus::bus_t& bus)
{
    return psu::ipmb::sendCommand(bus, psu::nm::getPsuConfigCmd,
                                  psu::nm::encodeGetPsuConfig());
}

} // namespace

int main()
{
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.psu-nm-proxy");

    sdbusplus::asio::object_server server(conn);

    // Startup gate: the ME must advertise the PMBUS proxy as supported (`01b`)
    // or supported and enabled (`11b`) before we publish (spec §5.2, §5.3).
    bool publishEnabled = false;
    try
    {
        publishEnabled = psu::nm::pmbusProxyEnabled(psu::ipmb::sendCommand(
            *conn, psu::nm::getFwCapabilitiesCmd,
            psu::nm::encodeGetFwCapabilities()));
    }
    catch (const std::exception& e)
    {
        std::cerr << "psu-nm-proxy: DEh capabilities read failed: " << e.what()
                  << "\n";
    }
    if (!publishEnabled)
    {
        std::cerr << "psu-nm-proxy: Intel ME PMBUS proxy not enabled; "
                     "publishing disabled\n";
    }

    // One poller per discovered slot publishes the §8 sensor set. Poller is
    // non-movable (it owns a steady_timer and references), so keep them behind
    // unique_ptr for the process lifetime.
    std::vector<std::unique_ptr<psu::sensors::Poller>> pollers;
    if (publishEnabled)
    {
        try
        {
            auto config = readPsuConfig(*conn);
            if (auto fallback = psu::nm::setPsuConfigFallback(
                    enableSetPsuConfigFallback, config, 0,
                    configuredPsuAddresses8))
            {
                psu::ipmb::sendCommand(*conn, psu::nm::setPsuConfigCmd,
                                       *fallback);
                config = readPsuConfig(*conn);
            }

            std::vector<std::pair<std::string, std::uint8_t>> slots;
            std::vector<psu::inventory::PsuInventory> inventory;
            for (const auto& slot : psu::nm::psuSlots(config))
            {
                const std::string name = "PSU" + std::to_string(slot.number);
                slots.emplace_back(name, slot.address8);

                // Asset strings are read once at startup; a slot whose MFR_ID
                // is unusable is published as absent (spec §5.6).
                inventory.push_back(psu::inventory::makeInventory(
                    name, psu::inventory::readAsset(*conn, slot.address8)));
            }

            // Publish inventory first so each poller can flip its slot's
            // Present/Functional on a failure-state transition (spec §5.7).
            const auto handles = psu::inventory::publish(server, inventory);

            for (const auto& [name, address8] : slots)
            {
                const auto slotHandles = handles.at(name);
                pollers.push_back(std::make_unique<psu::sensors::Poller>(
                    io, *conn, server, name, address8, pollInterval,
                    failuresBeforeNotFunctional,
                    [slotHandles](bool present, bool functional) {
                        psu::inventory::setPresentAndFunctional(
                            slotHandles, present, functional);
                    }));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "psu-nm-proxy: D8h slot discovery failed: " << e.what()
                      << "\n";
        }
    }

    // Poll unconditionally, in every host power state (including host-off).
    // Never gate on host state. Keep the process alive even when the proxy is
    // disabled and no poller exists.
    auto keepAlive = boost::asio::make_work_guard(io);
    for (auto& poller : pollers)
    {
        poller->start();
    }

    io.run();

    return 0;
}
