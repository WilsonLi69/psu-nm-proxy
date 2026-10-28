#include "config.hpp"
#include "dbus_adapters.hpp"
#include "poller.hpp"
#include "slot_discovery.hpp"

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <iostream>
#include <memory>
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

    auto server = std::make_shared<sdbusplus::asio::object_server>(conn);
    server->add_manager("/xyz/openbmc_project/inventory");
    server->add_manager("/xyz/openbmc_project/sensors");

    auto pollers =
        std::make_shared<std::vector<std::unique_ptr<psu::sensors::Poller>>>();

    auto nmClient =
        std::make_shared<psu::discovery::DbusNodeManagerClient>(*conn);
    auto mapperClient =
        std::make_shared<psu::discovery::DbusObjectMapperClient>(*conn);
    auto invSink =
        std::make_shared<psu::discovery::DbusInventorySink>(*server);

    auto discovery = std::make_shared<psu::discovery::SlotDiscovery>(
        io, config, *nmClient, *mapperClient, *invSink);

    discovery->start(
        [&io, conn, server, config, pollInterval, pollers,
         nmClient, mapperClient, invSink](
            psu::discovery::DiscoveryOutcome outcome) {
            if (outcome.status !=
                psu::discovery::DiscoveryOutcome::Status::success)
            {
                std::cerr << "psu-nm-proxy: " << outcome.message << "\n";
                return;
            }

            const auto sensorDefs = psu::config::selectSensors(config);
            for (const auto& slot : outcome.slots)
            {
                auto poller = std::make_unique<psu::sensors::Poller>(
                    io, *conn, *server, slot.name, slot.address8, pollInterval,
                    config.failuresBeforeNotFunctional, sensorDefs,
                    config.thresholds, slot.updateInventoryHealth,
                    slot.present, nmClient.get(), invSink.get());
                poller->start();
                pollers->push_back(std::move(poller));
            }
        });

    // Keep the process alive even when the proxy is disabled and no poller
    // exists, and run the io_context that serves D-Bus.
    auto keepAlive = boost::asio::make_work_guard(io);
    io.run();

    return 0;
}
