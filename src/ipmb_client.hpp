#pragma once

#include <sdbusplus/bus.hpp>

#include <cstdint>
#include <vector>

namespace psu::ipmb
{

inline constexpr auto service = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
inline constexpr auto path = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
inline constexpr auto interface = "org.openbmc.Ipmb";

// Issue one Intel Node Manager command (`cmd`) carrying `data` to the ME and
// return the response bytes, with the Intel manufacturer-ID echo stripped.
//
// Throws sdbusplus::exception::exception on a D-Bus failure and
// std::runtime_error on a non-success IPMB status / completion code.
std::vector<std::uint8_t> sendCommand(sdbusplus::bus_t& bus, std::uint8_t cmd,
                                      const std::vector<std::uint8_t>& data);

// Issue one Node Manager D9h "Send Raw PMBUS command" carrying `payload` and
// return the PMBus bytes, with the Intel manufacturer-ID echo stripped.
//
// Throws sdbusplus::exception::exception on a D-Bus failure and
// std::runtime_error on a non-success IPMB status / completion code.
std::vector<std::uint8_t>
    sendRawPmbus(sdbusplus::bus_t& bus,
                 const std::vector<std::uint8_t>& payload);

// As sendCommand, but a non-zero IPMI completion code is tolerated when the
// response still carries the Intel echo + data. Some ME firmware answers the
// MFR_ID/MFR_MODEL BLOCK_READ with completion code 0xA3 yet returns valid
// Asset strings. A response without the echo is still rejected, and the strict
// sendCommand/word-read path is unchanged.
std::vector<std::uint8_t>
    sendCommandToleratingCompletionCode(
        sdbusplus::bus_t& bus, std::uint8_t cmd,
        const std::vector<std::uint8_t>& data);

// As sendRawPmbus, tolerating a non-zero completion code (see above).
std::vector<std::uint8_t> sendRawPmbusToleratingCompletionCode(
    sdbusplus::bus_t& bus, const std::vector<std::uint8_t>& payload);

} // namespace psu::ipmb
