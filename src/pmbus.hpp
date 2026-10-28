#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace psu::pmbus
{

// Decode a PMBus Linear11 16-bit word: Value = Y * 2^N
//   N = signed [15:11] (5 bits), Y = signed [10:0] (11 bits).
inline double decodeLinear11(std::uint16_t word)
{
    constexpr std::uint16_t exponentMask = 0x1F;
    constexpr std::uint16_t mantissaMask = 0x7FF;
    constexpr std::uint16_t exponentSign = 0x10;
    constexpr std::uint16_t mantissaSign = 0x400;
    constexpr std::uint16_t exponentModulus = 0x20;
    constexpr std::uint16_t mantissaModulus = 0x800;

    const std::uint16_t rawExponent = (word >> 11) & exponentMask;
    const std::uint16_t rawMantissa = word & mantissaMask;

    const int exponent = (rawExponent & exponentSign)
                             ? static_cast<int>(rawExponent) - exponentModulus
                             : static_cast<int>(rawExponent);
    const int mantissa = (rawMantissa & mantissaSign)
                             ? static_cast<int>(rawMantissa) - mantissaModulus
                             : static_cast<int>(rawMantissa);

    return std::ldexp(static_cast<double>(mantissa), exponent);
}

// Decode a PMBus Linear16 word: Value = Y * 2^N, Y unsigned, N supplied
// separately (READ_VOUT takes N from VOUT_MODE).
inline double decodeLinear16(std::uint16_t mantissa, int exponent)
{
    return std::ldexp(static_cast<double>(mantissa), exponent);
}

// Extract the signed exponent N from a VOUT_MODE (0x20) byte: MODE is [7:5],
// the Linear16 exponent is signed [4:0]. 0x1C -> 0b11100 -> -4.
inline int voutModeExponent(std::uint8_t voutMode)
{
    constexpr std::uint8_t exponentMask = 0x1F;
    constexpr std::uint8_t exponentSign = 0x10;
    constexpr int exponentModulus = 0x20;

    const std::uint8_t raw = voutMode & exponentMask;
    return (raw & exponentSign) ? static_cast<int>(raw) - exponentModulus
                                : static_cast<int>(raw);
}

// Decode a length-prefixed PMBus ASCII block (MFR_ID / MFR_MODEL): the first
// byte is the payload length, the following bytes are the ASCII characters.
// Trailing bytes past the declared length are ignored.
inline std::string decodeAscii(const std::vector<std::uint8_t>& bytes)
{
    if (bytes.empty())
    {
        throw std::runtime_error("empty PMBus ASCII block");
    }

    const std::size_t length = bytes[0];
    if (bytes.size() < length + 1)
    {
        throw std::runtime_error("short PMBus ASCII block");
    }

    return {bytes.begin() + 1, bytes.begin() + 1 + length};
}

// Critical PMBus STATUS_WORD hardware fault mask (spec §5.5/§5.7, ADR-0003):
// [2] TEMPERATURE_FAULT, [8] VOUT_OV_FAULT, [9] IOUT_OC_FAULT, [10] FAN_FAULT,
// [13] VIN_UV_FAULT.
inline constexpr std::uint16_t statusWordCriticalFaultMask = 0x2704;

inline constexpr std::uint16_t statusWordCmlBit = 1;
inline constexpr std::uint16_t statusWordTemperatureFaultBit = 2;
inline constexpr std::uint16_t statusWordOffBit = 6;
inline constexpr std::uint16_t statusWordVoutOvFaultBit = 8;
inline constexpr std::uint16_t statusWordIoutOcFaultBit = 9;
inline constexpr std::uint16_t statusWordFanFaultBit = 10;
inline constexpr std::uint16_t statusWordPowerGoodBit = 11;
inline constexpr std::uint16_t statusWordVinUvFaultBit = 13;

// Decoded STATUS_WORD (0x79) operational and critical fault bits.
struct StatusWord
{
    std::uint16_t raw;
    bool off;
    bool powerGood;
    bool temperatureFault;
    bool voutOvFault;
    bool ioutOcFault;
    bool fanFault;
    bool vinUvFault;

    StatusWord() :
        raw(0), off(false), powerGood(false), temperatureFault(false),
        voutOvFault(false), ioutOcFault(false), fanFault(false),
        vinUvFault(false)
    {}

    StatusWord(bool off, bool powerGood) :
        raw(static_cast<std::uint16_t>(
            (off ? (1 << statusWordOffBit) : 0) |
            (powerGood ? 0 : (1 << statusWordPowerGoodBit)))),
        off(off), powerGood(powerGood), temperatureFault(false),
        voutOvFault(false), ioutOcFault(false), fanFault(false),
        vinUvFault(false)
    {}

    StatusWord(std::uint16_t raw, bool off, bool powerGood,
               bool temperatureFault, bool voutOvFault, bool ioutOcFault,
               bool fanFault, bool vinUvFault) :
        raw(raw),
        off(off), powerGood(powerGood), temperatureFault(temperatureFault),
        voutOvFault(voutOvFault), ioutOcFault(ioutOcFault), fanFault(fanFault),
        vinUvFault(vinUvFault)
    {}

    bool hasCriticalFault() const
    {
        return (raw & statusWordCriticalFaultMask) != 0;
    }

    bool operator==(const StatusWord&) const = default;
};

// STATUS_WORD bit field: [6] OFF (output disabled), [11] POWER_GOOD#
// (active-low: 0 indicates asserted/good, 1 indicates negated/not good),
// and the 5 critical hardware fault bits in mask 0x2704:
// [2] TEMPERATURE_FAULT, [8] VOUT_OV_FAULT, [9] IOUT_OC_FAULT, [10] FAN_FAULT,
// [13] VIN_UV_FAULT.
inline StatusWord decodeStatusWord(std::uint16_t word)
{
    return {
        word,
        ((word >> statusWordOffBit) & 0x1) != 0,
        ((word >> statusWordPowerGoodBit) & 0x1) == 0,
        ((word >> statusWordTemperatureFaultBit) & 0x1) != 0,
        ((word >> statusWordVoutOvFaultBit) & 0x1) != 0,
        ((word >> statusWordIoutOcFaultBit) & 0x1) != 0,
        ((word >> statusWordFanFaultBit) & 0x1) != 0,
        ((word >> statusWordVinUvFaultBit) & 0x1) != 0,
    };
}

inline bool hasCriticalFault(std::uint16_t word)
{
    return (word & statusWordCriticalFaultMask) != 0;
}

inline bool hasCriticalFault(const StatusWord& status)
{
    return status.hasCriticalFault();
}

inline std::string_view criticalFaultName(const StatusWord& status)
{
    if (status.temperatureFault)
    {
        return "TEMPERATURE_FAULT";
    }
    if (status.voutOvFault)
    {
        return "VOUT_OV_FAULT";
    }
    if (status.ioutOcFault)
    {
        return "IOUT_OC_FAULT";
    }
    if (status.fanFault)
    {
        return "FAN_FAULT";
    }
    if (status.vinUvFault)
    {
        return "VIN_UV_FAULT";
    }
    return {};
}

inline std::string_view criticalFaultName(std::uint16_t word)
{
    return criticalFaultName(decodeStatusWord(word));
}

inline std::string_view firstCriticalFaultName(const StatusWord& status)
{
    return criticalFaultName(status);
}

inline std::string_view firstCriticalFaultName(std::uint16_t word)
{
    return criticalFaultName(decodeStatusWord(word));
}

} // namespace psu::pmbus
