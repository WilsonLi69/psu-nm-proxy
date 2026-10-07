#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace psu::nm
{

// Intel Node Manager IPMI command / NetFn, and the D9h "Send Raw PMBUS
// command" data payload layout (docs/research/07-intel-me-node-manager-psu.md,
// docs/research/10-acbel-psu-command-set.md).
inline constexpr std::uint8_t netFn = 0x2E;
inline constexpr std::uint8_t sendRawPmbus = 0xD9;

// D9h flags: [5:4] standard device address, [3:1] transaction type READ_WORD.
inline constexpr std::uint8_t readWordFlags = 0x06;
inline constexpr std::uint8_t pmbusProtocol = 0x00;
inline constexpr std::uint8_t writeLength = 0x01;
inline constexpr std::uint8_t wordReadLength = 0x02;

// Intel manufacturer-ID echo the ME prefixes to a D9h response.
inline constexpr std::uint8_t intelEcho[] = {0x87, 0x01, 0x00};

// Build the ten-byte D9h data payload for a PMBus READ_WORD (observed on
// hardware: encodeReadWord(0xB2, 0x88) == 57 01 00 06 B2 00 00 01 02 88).
inline std::vector<std::uint8_t> encodeReadWord(std::uint8_t psuAddr8,
                                                std::uint8_t command)
{
    return {
        0x57, 0x01, 0x00, // Intel IANA 0x000157, little-endian
        readWordFlags,    // standard address, READ_WORD
        psuAddr8,         // target PSU 8-bit SMBus address
        0x00,             // MGPIO MUX
        pmbusProtocol,    // PMBus
        writeLength,      // command byte is written
        wordReadLength,   // two-byte read
        command,          // PMBus command byte
    };
}

// Strip the Intel manufacturer-ID echo from a D9h response, returning the raw
// PMBus payload bytes.
inline std::vector<std::uint8_t>
    stripIntelEcho(const std::vector<std::uint8_t>& response)
{
    if (response.size() < sizeof(intelEcho) ||
        !std::equal(std::begin(intelEcho), std::end(intelEcho),
                    response.begin()))
    {
        throw std::runtime_error(
            "D9h response missing Intel manufacturer-ID echo");
    }
    return {response.begin() + sizeof(intelEcho), response.end()};
}

} // namespace psu::nm
