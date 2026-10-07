#include "ipmb_client.hpp"

#include "nm_command.hpp"

#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace psu::ipmb
{

namespace
{

constexpr std::uint8_t ipmbChannel = 0x00; // 0 = IPMB
constexpr std::uint8_t lun = 0x00;
constexpr std::uint64_t callTimeoutUs = 2'000'000; // sendRequest retries

} // namespace

std::vector<std::uint8_t>
    sendRawPmbus(sdbusplus::bus_t& bus,
                 const std::vector<std::uint8_t>& payload)
{
    auto method = bus.new_method_call(service, path, interface, "sendRequest");
    method.append(ipmbChannel, psu::nm::netFn, lun, psu::nm::sendRawPmbus,
                  payload);

    auto reply = bus.call(method, callTimeoutUs);

    int status = 0;
    std::uint8_t netFn = 0;
    std::uint8_t rsLun = 0;
    std::uint8_t cmd = 0;
    std::uint8_t completionCode = 0;
    std::vector<std::uint8_t> data;
    reply.read(status, netFn, rsLun, cmd, completionCode, data);

    if (status != 0 || completionCode != 0x00)
    {
        throw std::runtime_error(
            "IPMB sendRequest failed (status=" + std::to_string(status) +
            ", completionCode=" + std::to_string(completionCode) + ")");
    }

    return psu::nm::stripIntelEcho(data);
}

} // namespace psu::ipmb
