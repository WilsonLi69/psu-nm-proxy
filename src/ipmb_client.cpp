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

std::vector<std::uint8_t>
    sendCommandImpl(sdbusplus::bus_t& bus, std::uint8_t cmd,
                    const std::vector<std::uint8_t>& data, bool tolerateCc)
{
    auto method = bus.new_method_call(service, path, interface, "sendRequest");
    method.append(channel, psu::nm::netFn, lun, cmd, data);

    auto reply = bus.call(method, callTimeoutUs);

    using IpmbReply = std::tuple<int, std::uint8_t, std::uint8_t, std::uint8_t,
                                  std::uint8_t, std::vector<std::uint8_t>>;
    IpmbReply replyFields;
    reply.read(replyFields);
    auto [status, rsNetFn, rsLun, rsCmd, completionCode, response] = replyFields;

    if (status != 0 || (!tolerateCc && completionCode != 0x00))
    {
        throw std::runtime_error(
            "IPMB sendRequest failed (status=" + std::to_string(status) +
            ", completionCode=" + std::to_string(completionCode) + ")");
    }

    return psu::nm::stripIntelEcho(response);
}

} // namespace

std::vector<std::uint8_t> sendCommand(sdbusplus::bus_t& bus, std::uint8_t cmd,
                                      const std::vector<std::uint8_t>& data)
{
    return sendCommandImpl(bus, cmd, data, false);
}

std::vector<std::uint8_t>
    sendCommandToleratingCompletionCode(
        sdbusplus::bus_t& bus, std::uint8_t cmd,
        const std::vector<std::uint8_t>& data)
{
    return sendCommandImpl(bus, cmd, data, true);
}

std::vector<std::uint8_t>
    sendRawPmbus(sdbusplus::bus_t& bus,
                 const std::vector<std::uint8_t>& payload)
{
    return sendCommand(bus, psu::nm::sendRawPmbusCmd, payload);
}

std::vector<std::uint8_t> sendRawPmbusToleratingCompletionCode(
    sdbusplus::bus_t& bus, const std::vector<std::uint8_t>& payload)
{
    return sendCommandToleratingCompletionCode(bus, psu::nm::sendRawPmbusCmd,
                                               payload);
}

} // namespace psu::ipmb
