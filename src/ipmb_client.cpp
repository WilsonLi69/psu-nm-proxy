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

constexpr std::uint8_t channel = 0x01; // 0 = IPMB; 1 = ME
constexpr std::uint8_t lun = 0x00;
constexpr std::uint64_t callTimeoutUs = 2'000'000; // sendRequest retries

} // namespace

std::vector<std::uint8_t> sendCommand(sdbusplus::bus_t& bus, std::uint8_t cmd,
                                      const std::vector<std::uint8_t>& data)
{
    auto method = bus.new_method_call(service, path, interface, "sendRequest");
    method.append(channel, psu::nm::netFn, lun, cmd, data);

    auto reply = bus.call(method, callTimeoutUs);

    int status = 0;
    std::uint8_t rsNetFn = 0;
    std::uint8_t rsLun = 0;
    std::uint8_t rsCmd = 0;
    std::uint8_t completionCode = 0;
    std::vector<std::uint8_t> response;
    reply.read(status, rsNetFn, rsLun, rsCmd, completionCode, response);

    if (status != 0 || completionCode != 0x00)
    {
        throw std::runtime_error(
            "IPMB sendRequest failed (status=" + std::to_string(status) +
            ", completionCode=" + std::to_string(completionCode) + ")");
    }

    return psu::nm::stripIntelEcho(response);
}

std::vector<std::uint8_t>
    sendRawPmbus(sdbusplus::bus_t& bus,
                 const std::vector<std::uint8_t>& payload)
{
    return sendCommand(bus, psu::nm::sendRawPmbusCmd, payload);
}

} // namespace psu::ipmb
