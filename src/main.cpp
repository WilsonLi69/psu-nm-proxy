#include "ipmb_client.hpp"
#include "linear11.hpp"
#include "nm_command.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

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

constexpr const char* sensorPath =
    "/xyz/openbmc_project/sensors/voltage/PSU2_VIN";
constexpr const char* sensorInterface = "xyz.openbmc_project.Sensor.Value";
constexpr const char* statusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
constexpr const char* voltsUnit =
    "xyz.openbmc_project.Sensor.Value.Unit.Volts";

// Slot 2 is the only installed PSU; discovery is ticket #16.
constexpr std::uint8_t psuAddress8 = 0xB2;
constexpr std::uint8_t readVin = 0x88;
constexpr auto pollInterval = std::chrono::seconds(1);

} // namespace

int main()
{
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.psu-nm-proxy");

    sdbusplus::asio::object_server server(conn);

    auto sensor = server.add_interface(sensorPath, sensorInterface);
    sensor->register_property("Value", 0.0);
    sensor->register_property("Unit", std::string(voltsUnit));
    sensor->initialize();

    auto status = server.add_interface(sensorPath, statusInterface);
    status->register_property("Functional", true);
    status->initialize();

    // Poll unconditionally, in every host power state (including host-off).
    // Never gate on host state.
    boost::asio::steady_timer timer(io);
    std::function<void()> poll;
    poll = [&]() {
        try
        {
            const auto payload = psu::nm::encodeReadWord(psuAddress8, readVin);
            const auto bytes = psu::ipmb::sendRawPmbus(*conn, payload);
            if (bytes.size() < 2)
            {
                throw std::runtime_error("short READ_VIN response (" +
                                         std::to_string(bytes.size()) +
                                         " bytes)");
            }

            const auto word = static_cast<std::uint16_t>(bytes[0]) |
                              (static_cast<std::uint16_t>(bytes[1]) << 8);
            const double volts = psu::pmbus::decodeLinear11(word);

            sensor->set_property("Value", volts);
            status->set_property("Functional", true);
            std::cout << "psu-nm-proxy: READ_VIN = " << volts << " V\n";
        }
        catch (const std::exception& e)
        {
            status->set_property("Functional", false);
            std::cerr << "psu-nm-proxy: READ_VIN failed: " << e.what() << "\n";
        }

        timer.expires_after(pollInterval);
        timer.async_wait([&](const boost::system::error_code&) { poll(); });
    };

    poll();
    io.run();

    return 0;
}
