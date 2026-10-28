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

// Intel Node Manager standard completion codes (spec §2.1/§5.3).
inline constexpr std::uint8_t ccSuccess = 0x00;
inline constexpr std::uint8_t ccDeviceTimeout = 0x80;
inline constexpr std::uint8_t ccNodeBusy = 0xC0;
inline constexpr std::uint8_t ccReadLengthDifferent = 0xA3;

// Response bytes from the ME, with the Intel manufacturer-ID echo stripped, or
// no value when the asynchronous call failed, timed out, or reported a
// non-success IPMB status / completion code.
using Response = std::optional<std::vector<std::uint8_t>>;

// Invoked on the io_context thread once an asynchronous command completes.
using ResponseCallback = std::function<void(Response)>;

// Low-level raw PMBus command outcome preserving the completion code and
// transport status for higher-level discrimination (spec §5.3).
struct RawPmbusOutcome
{
    std::uint8_t completionCode = 0xFF;
    bool transportOk = false;
    std::optional<std::vector<std::uint8_t>> payload = std::nullopt;
};

using RawPmbusCallback = std::function<void(RawPmbusOutcome)>;

// Normalizes Intel ME completion code 0xA3 on PMBus BLOCK_READ asset transactions
// (e.g. MFR_ID / MFR_MODEL) with valid Intel echo and ASCII payload directly to
// Success (0x00), keeping WordReader and higher-level domain pollers decoupled
// from 0xA3.
std::uint8_t normalizeCompletionCode(std::uint8_t completionCode,
                                     const std::vector<std::uint8_t>& response);

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
// response still carries the Intel echo + data.
void sendCommandToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn, std::uint8_t cmd,
    const std::vector<std::uint8_t>& data, ResponseCallback callback);

// Issue one Node Manager D9h "Send Raw PMBUS command" carrying `payload`.
void sendRawPmbusAsync(sdbusplus::asio::connection& conn,
                       const std::vector<std::uint8_t>& payload,
                       RawPmbusCallback callback);

// Overload delivering stripped payload Response on success, std::nullopt on failure.
void sendRawPmbusAsync(sdbusplus::asio::connection& conn,
                       const std::vector<std::uint8_t>& payload,
                       ResponseCallback callback);

// As sendRawPmbusAsync, tolerating a non-zero completion code (see above).
void sendRawPmbusToleratingCompletionCodeAsync(
    sdbusplus::asio::connection& conn,
    const std::vector<std::uint8_t>& payload, ResponseCallback callback);

} // namespace psu::ipmb
