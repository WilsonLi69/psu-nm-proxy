#include "poller.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>

namespace
{

// Slot 2 is the only installed PSU; discovery is ticket #16.
constexpr std::uint8_t psuAddress8 = 0xB2;
constexpr auto pollInterval = std::chrono::seconds(1);

} // namespace

int main()
{
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.psu-nm-proxy");

    sdbusplus::asio::object_server server(conn);

    psu::sensors::Poller poller(io, *conn, server, "PSU2", psuAddress8,
                                pollInterval);
    poller.start();

    io.run();

    return 0;
}
