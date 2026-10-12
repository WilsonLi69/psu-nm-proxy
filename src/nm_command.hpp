#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace psu::nm
{

// Intel Node Manager IPMI command / NetFn, and the D9h "Send Raw PMBUS
// command" data payload layout (docs/research/07-intel-me-node-manager-psu.md,
// docs/research/10-acbel-psu-command-set.md).
inline constexpr std::uint8_t netFn = 0x2E;
inline constexpr std::uint8_t sendRawPmbusCmd = 0xD9;
inline constexpr std::uint8_t getFwCapabilitiesCmd = 0xDE;
inline constexpr std::uint8_t getPsuConfigCmd = 0xD8;
inline constexpr std::uint8_t setPsuConfigCmd = 0xD7;

// D9h flags: [5:4] standard device address, [3:1] transaction type READ_WORD.
inline constexpr std::uint8_t readWordFlags = 0x06;
// BLOCK_READ transaction type (observed on the board for MFR_ID/MFR_MODEL).
inline constexpr std::uint8_t blockReadFlags = 0x0A;
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

// Build the ten-byte D9h data payload for a PMBus BLOCK_READ (observed on
// hardware for MFR_ID: encodeBlockRead(0xB2, 0x99, 0x10) ==
// 57 01 00 0a B2 00 00 01 10 99). The read length is the number of bytes the
// ME is asked to return (e.g. 0x10 = 16 covers both MFR strings).
inline std::vector<std::uint8_t>
    encodeBlockRead(std::uint8_t psuAddr8, std::uint8_t command,
                    std::uint8_t readLength)
{
    return {
        0x57, 0x01, 0x00, // Intel IANA 0x000157, little-endian
        blockReadFlags,   // standard address, BLOCK_READ
        psuAddr8,         // target PSU 8-bit SMBus address
        0x00,             // MGPIO MUX
        pmbusProtocol,    // PMBus
        writeLength,      // command byte is written
        readLength,       // caller-supplied block read length
        command,          // PMBus command byte
    };
}

// Build the eleven-byte DEh "Get Intel ME FW Capabilities" data payload.
inline std::vector<std::uint8_t> encodeGetFwCapabilities()
{
    return {
        0x57, 0x01, 0x00, // Intel IANA 0x000157, little-endian
        0x00, 0x00, 0x00, 0x00, // reserved
        0x02,             // IPMI version 2.0
        0x20, 0x00,       // max message size 0x0020
        0x3F,             // BMC proxy bitmap
    };
}

// The DEh capability byte sits at payload index 8 after the Intel echo. Its
// bits [3:2] advertise PMBUS proxy support: 00b unsupported, 01b supported,
// 11b supported and enabled. A missing/short byte is unsupported.
inline bool
    pmbusProxyEnabled(const std::vector<std::uint8_t>& payloadAfterEcho)
{
    if (payloadAfterEcho.size() <= 8)
    {
        return false;
    }
    const std::uint8_t proxy = (payloadAfterEcho[8] >> 2) & 0x3;
    return proxy == 0x1 || proxy == 0x3;
}

// Build the four-byte D8h "Get PSU Configuration" data payload.
inline std::vector<std::uint8_t> encodeGetPsuConfig(std::uint8_t domain = 0)
{
    return {0x57, 0x01, 0x00, domain};
}

// One configured slot from a D8h response: the 1-based slot number (payload
// index) and its 8-bit PSU SMBus address (`byte & 0xFE`, mode bit dropped).
struct PsuSlot
{
    std::uint8_t number;
    std::uint8_t address8;

    bool operator==(const PsuSlot&) const = default;
};

// Parse the non-zero slots out of a D8h response (payload index 1..8 map to
// slots 1..8; payload index 0 is the domain).
inline std::vector<PsuSlot>
    psuSlots(const std::vector<std::uint8_t>& payloadAfterEcho)
{
    std::vector<PsuSlot> slots;
    const std::size_t end =
        std::min<std::size_t>(payloadAfterEcho.size(), 9);
    for (std::size_t index = 1; index < end; ++index)
    {
        const std::uint8_t raw = payloadAfterEcho[index];
        if (raw != 0)
        {
            slots.push_back(
                {static_cast<std::uint8_t>(index),
                 static_cast<std::uint8_t>(raw & 0xFE)});
        }
    }
    return slots;
}

// Build the D7h "Set PSU Configuration" data payload from eight 8-bit PSU
// SMBus addresses (the `[7:1]` address plus a zero mode bit).
inline std::vector<std::uint8_t>
    encodeSetPsuConfig(std::uint8_t domain,
                       const std::array<std::uint8_t, 8>& addrs8)
{
    std::vector<std::uint8_t> payload = {0x57, 0x01, 0x00, domain};
    payload.insert(payload.end(), addrs8.begin(), addrs8.end());
    return payload;
}

// The D7h fallback payload, or no value when it must not be written: it is
// only ever used when explicitly enabled AND D8h reported no configured slot
// (spec §5.3, §5.2). Off by default.
inline std::optional<std::vector<std::uint8_t>>
    setPsuConfigFallback(bool enabled,
                         const std::vector<std::uint8_t>& d8PayloadAfterEcho,
                         std::uint8_t domain,
                         const std::array<std::uint8_t, 8>& addrs8)
{
    if (!enabled || !psuSlots(d8PayloadAfterEcho).empty())
    {
        return std::nullopt;
    }
    return encodeSetPsuConfig(domain, addrs8);
}

// True when `response` begins with the Intel manufacturer-ID echo. The ME
// prefixes this even on responses that carry a non-zero completion code, so
// callers can use it to tell "echo + data" apart from a genuinely failed read.
inline bool hasIntelEcho(const std::vector<std::uint8_t>& response)
{
    return response.size() >= sizeof(intelEcho) &&
           std::equal(std::begin(intelEcho), std::end(intelEcho),
                      response.begin());
}

// Strip the Intel manufacturer-ID echo from a D9h response, returning the raw
// PMBus payload bytes.
inline std::vector<std::uint8_t>
    stripIntelEcho(const std::vector<std::uint8_t>& response)
{
    if (!hasIntelEcho(response))
    {
        throw std::runtime_error(
            "D9h response missing Intel manufacturer-ID echo");
    }
    return {response.begin() + sizeof(intelEcho), response.end()};
}

// Assemble the little-endian 16-bit PMBus word at `offset` in `response`.
inline std::uint16_t
    wordFromLittleEndian(const std::vector<std::uint8_t>& response,
                         std::size_t offset = 0)
{
    if (response.size() < offset + 2)
    {
        throw std::runtime_error("short PMBus word response");
    }
    return static_cast<std::uint16_t>(response[offset]) |
           (static_cast<std::uint16_t>(response[offset + 1]) << 8);
}

} // namespace psu::nm
