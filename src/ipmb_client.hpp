#pragma once

#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace psu::ipmb
{

inline constexpr auto service = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
inline constexpr auto path = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
inline constexpr auto interface = "org.openbmc.Ipmb";

// Response bytes from the ME, with the Intel manufacturer-ID echo stripped, or
// no value when the asynchronous call failed, timed out, or reported a
// non-success IPMB status / completion code.
using Response = std::optional<std::vector<std::uint8_t>>;

// Invoked on the io_context thread once an asynchronous command completes.
using ResponseCallback = std::function<void(Response)>;

// Issue one Intel Node Manager command (`cmd`) carrying `data` to the ME.
//
// All of these are asynchronous: they return immediately and never block the
// io_context thread while the ME/IPMB is slow or absent, so the object server
// keeps answering introspection and property reads throughout a poll. The
// callback always runs later, on the io_context thread.
//
// Each callback yields the response bytes (Intel echo stripped) on success, or
// std::nullopt on a D-Bus failure, a timeout, a non-success IPMB status /
// completion code, or a malformed response.
void sendCommandAsync(sdbusplus::asio::connection& conn, std::uint8_t cmd,
                      const std::vector<std::uint8_t>& data,
                      ResponseCallback callback);

// As sendCommandAsync, but tolerates a non-zero IPMI completion code when the
// response still carries the Intel echo + data. Some ME firmware answers the
// MFR_ID/MFR_MODEL BLOCK_READ with completion code 0xA3 yet returns valid
// Asset strings. A response without the echo is still rejected.
void sendCommandToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn, std::uint8_t cmd,
    const std::vector<std::uint8_t>& data, ResponseCallback callback);

// Issue one Node Manager D9h "Send Raw PMBUS command" carrying `payload`.
void sendRawPmbusAsync(sdbusplus::asio::connection& conn,
                       const std::vector<std::uint8_t>& payload,
                       ResponseCallback callback);

// As sendRawPmbusAsync, tolerating a non-zero completion code (see above).
void sendRawPmbusToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn,
    const std::vector<std::uint8_t>& payload, ResponseCallback callback);

} // namespace psu::ipmb
