#include "config.hpp"
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

// D7h is a fallback only and is never written by default (spec §5.3, §5.2).
constexpr bool enableSetPsuConfigFallback = false;
constexpr std::array<std::uint8_t, 8> configuredPsuAddresses8 = {};

std::vector<std::uint8_t> readPsuConfig(sdbusplus::bus_t& bus)
{
    return psu::ipmb::sendCommand(bus, psu::nm::getPsuConfigCmd,
                                  psu::nm::encodeGetPsuConfig());
}

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

} // namespace

int main(int argc, char* argv[])
{
    const std::string configPath =
        argc > 1 ? argv[1] : std::string(psu::config::defaultConfigPath);
    const auto config = psu::config::loadConfig(configPath);
    logConfig(config);

    const auto pollInterval =
        std::chrono::milliseconds(config.pollIntervalMs);

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
            auto psuConfig = readPsuConfig(*conn);
            if (auto fallback = psu::nm::setPsuConfigFallback(
                    enableSetPsuConfigFallback, psuConfig, 0,
                    configuredPsuAddresses8))
            {
                psu::ipmb::sendCommand(*conn, psu::nm::setPsuConfigCmd,
                                       *fallback);
                psuConfig = readPsuConfig(*conn);
            }

            // expectedSlots/psuNames drive discovery matching and slot naming
            // (spec §6).
            const auto resolved =
                psu::config::resolveSlots(config, psu::nm::psuSlots(psuConfig));

            std::vector<std::pair<std::string, std::uint8_t>> slots;
            std::vector<psu::inventory::PsuInventory> inventory;
            for (const auto& slot : resolved)
            {
                slots.emplace_back(slot.name, slot.address8);

                // Asset strings are read once at startup; a slot whose MFR_ID
                // is unusable is published as absent (spec §5.6).
                inventory.push_back(psu::inventory::makeInventory(
                    slot.name, psu::inventory::readAsset(*conn, slot.address8)));
            }

            // Publish inventory first so each poller can flip its slot's
            // Present/Functional on a failure-state transition (spec §5.7).
            const auto handles = psu::inventory::publish(server, inventory);
            const auto sensorDefs = psu::config::selectSensors(config);

            for (const auto& [name, address8] : slots)
            {
                const auto slotHandles = handles.at(name);
                pollers.push_back(std::make_unique<psu::sensors::Poller>(
                    io, *conn, server, name, address8, pollInterval,
                    config.failuresBeforeNotFunctional, sensorDefs,
                    config.thresholds,
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
