#pragma once

#include <sdbusplus/bus.hpp>

#include <cstdint>
#include <vector>

namespace psu::ipmb
{

inline constexpr auto service = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
inline constexpr auto path = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
inline constexpr auto interface = "org.openbmc.Ipmb";

// Issue one Node Manager D9h "Send Raw PMBUS command" carrying `payload` and
// return the PMBus bytes, with the Intel manufacturer-ID echo stripped.
//
// Throws sdbusplus::exception::exception on a D-Bus failure and
// std::runtime_error on a non-success IPMB status / completion code.
std::vector<std::uint8_t>
    sendRawPmbus(sdbusplus::bus_t& bus,
                 const std::vector<std::uint8_t>& payload);

} // namespace psu::ipmb
