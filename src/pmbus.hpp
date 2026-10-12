#pragma once

#include <cmath>
#include <cstdint>

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

} // namespace psu::pmbus
