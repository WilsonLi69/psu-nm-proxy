#include "ipmb_client.hpp"
#include "linear11.hpp"
#include "nm_command.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr const char* sensorPathPrefix =
    "/xyz/openbmc_project/sensors/voltage/";
constexpr const char* sensorInterface = "xyz.openbmc_project.Sensor.Value";
constexpr const char* statusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
constexpr const char* voltsUnit =
    "xyz.openbmc_project.Sensor.Value.Unit.Volts";

constexpr std::uint8_t readVin = 0x88;
constexpr auto pollInterval = std::chrono::seconds(1);

// D7h is a fallback only and is never written by default (spec §5.3, §5.2).
constexpr bool enableSetPsuConfigFallback = false;
constexpr std::array<std::uint8_t, 8> configuredPsuAddresses8 = {};

struct PublishedPsu
{
    std::uint8_t address8;
    std::shared_ptr<sdbusplus::asio::dbus_interface> value;
    std::shared_ptr<sdbusplus::asio::dbus_interface> status;
};

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

    std::vector<PublishedPsu> psus;
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

            // Represent every configured slot; a registered-but-empty slot is
            // published with Functional=false (inventory is ticket #18).
            for (const auto& slot : psu::nm::psuSlots(config))
            {
                const std::string path = std::string(sensorPathPrefix) + "PSU" +
                                         std::to_string(slot.number) + "_VIN";

                auto value = server.add_interface(path, sensorInterface);
                value->register_property("Value", 0.0);
                value->register_property("Unit", std::string(voltsUnit));
                value->initialize();

                auto status = server.add_interface(path, statusInterface);
                status->register_property("Functional", false);
                status->initialize();

                psus.push_back({slot.address8, value, status});
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "psu-nm-proxy: D8h slot discovery failed: " << e.what()
                      << "\n";
        }
    }

    // Poll unconditionally, in every host power state (including host-off).
    // Never gate on host state.
    boost::asio::steady_timer timer(io);
    std::function<void()> poll;
    poll = [&]() {
        for (auto& psu : psus)
        {
            try
            {
                const auto bytes = psu::ipmb::sendRawPmbus(
                    *conn, psu::nm::encodeReadWord(psu.address8, readVin));
                const double volts = psu::pmbus::decodeLinear11(
                    psu::nm::wordFromLittleEndian(bytes));

                psu.value->set_property("Value", volts);
                psu.status->set_property("Functional", true);
            }
            catch (const std::exception& e)
            {
                psu.status->set_property("Functional", false);
                std::cerr << "psu-nm-proxy: READ_VIN @0x" << std::hex
                          << static_cast<int>(psu.address8) << std::dec
                          << " failed: " << e.what() << "\n";
            }
        }

        timer.expires_after(pollInterval);
        timer.async_wait([&](const boost::system::error_code&) { poll(); });
    };

    poll();
    io.run();

    return 0;
}
