#include "ipmb_client.hpp"

#include "nm_command.hpp"

#include <boost/asio/post.hpp>

#include <cstdint>
#include <exception>
#include <tuple>
#include <utility>
#include <vector>

namespace psu::ipmb
{

namespace
{

constexpr std::uint8_t channel = 0x01; // 0 = IPMB; 1 = ME
constexpr std::uint8_t lun = 0x00;
constexpr std::uint64_t callTimeoutUs = 2'000'000; // async method-call timeout

using IpmbReply = std::tuple<int, std::uint8_t, std::uint8_t, std::uint8_t,
                             std::uint8_t, std::vector<std::uint8_t>>;

void sendCommandImpl(sdbusplus::asio::connection& conn, std::uint8_t cmd,
                     const std::vector<std::uint8_t>& data, bool tolerateCc,
                     ResponseCallback callback)
{
    conn.async_method_call_timed(
        [&conn, tolerateCc, callback = std::move(callback)](
            const boost::system::error_code& ec, const IpmbReply& replyFields) {
            Response result;
            if (!ec)
            {
                const auto& [status, rsNetFn, rsLun, rsCmd, completionCode,
                             response] = replyFields;
                if (status == 0 && (tolerateCc || completionCode == 0x00))
                {
                    try
                    {
                        result = psu::nm::stripIntelEcho(response);
                    }
                    catch (const std::exception&)
                    {
                        result = std::nullopt;
                    }
                }
            }

            // The callback must always run later, on the io_context thread:
            // sd-bus delivers some failures (and argument errors) synchronously
            // from within sendCommandImpl, and callers rely on the callback
            // not firing before they have finished setting up.
            boost::asio::post(
                conn.get_io_context(),
                [callback = std::move(callback),
                 result = std::move(result)]() mutable {
                    callback(std::move(result));
                });
        },
        service, path, interface, "sendRequest", callTimeoutUs, channel,
        psu::nm::netFn, lun, cmd, data);
}

} // namespace

void sendCommandAsync(sdbusplus::asio::connection& conn, std::uint8_t cmd,
                      const std::vector<std::uint8_t>& data,
                      ResponseCallback callback)
{
    sendCommandImpl(conn, cmd, data, false, std::move(callback));
}

void sendCommandToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn, std::uint8_t cmd,
    const std::vector<std::uint8_t>& data, ResponseCallback callback)
{
    sendCommandImpl(conn, cmd, data, true, std::move(callback));
}

void sendRawPmbusAsync(sdbusplus::asio::connection& conn,
                       const std::vector<std::uint8_t>& payload,
                       ResponseCallback callback)
{
    sendCommandAsync(conn, psu::nm::sendRawPmbusCmd, payload,
                     std::move(callback));
}

void sendRawPmbusToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn,
    const std::vector<std::uint8_t>& payload, ResponseCallback callback)
{
    sendCommandToleratingCompletionCodeAsync(conn, psu::nm::sendRawPmbusCmd,
                                             payload, std::move(callback));
}

} // namespace psu::ipmb
