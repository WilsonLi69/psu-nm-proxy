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

} // namespace psu::pmbus
