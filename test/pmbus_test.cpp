#include "pmbus.hpp"

#include <gtest/gtest.h>

#include <cstdint>

// Linear16: READ_VOUT mantissa scaled by the exponent read from VOUT_MODE.
TEST(PmbusLinear16, DecodesProbedVout)
{
    // 0x00C3 with N = -4 -> 195 / 16 = 12.1875 V.
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear16(0x00C3, -4), 12.1875);
}

TEST(PmbusLinear16, ZeroDecodesToZero)
{
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear16(0x0000, -4), 0.0);
}

TEST(PmbusLinear16, MantissaIsUnsigned)
{
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear16(0xFFFF, 0), 65535.0);
}

TEST(PmbusLinear16, PositiveExponentScalesUp)
{
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear16(0x0003, 2), 12.0);
}

// VOUT_MODE 0x1C: MODE [7:5] = 000 (Linear), exponent [4:0] = 0b11100 = -4.
TEST(PmbusVoutMode, ExtractsProbedNegativeExponent)
{
    EXPECT_EQ(psu::pmbus::voutModeExponent(0x1C), -4);
}

TEST(PmbusVoutMode, ExtractsPositiveExponent)
{
    EXPECT_EQ(psu::pmbus::voutModeExponent(0x05), 5);
}

TEST(PmbusVoutMode, ExtractsMostNegativeExponent)
{
    // [4:0] = 0b10000 = -16.
    EXPECT_EQ(psu::pmbus::voutModeExponent(0x10), -16);
}

TEST(PmbusVoutMode, IgnoresModeBits)
{
    // MODE [7:5] set; only the exponent field is decoded.
    EXPECT_EQ(psu::pmbus::voutModeExponent(0xFC), -4);
}
