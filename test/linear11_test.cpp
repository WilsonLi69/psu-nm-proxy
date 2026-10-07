#include "linear11.hpp"

#include <gtest/gtest.h>

// Fixtures from the hardware probe recorded in
// docs/research/10-acbel-psu-command-set.md.
TEST(Linear11, DecodesProbedValues)
{
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x007A), 122.0);      // READ_VIN
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xD04B), 1.171875);   // READ_IIN
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xE855), 10.625);     // READ_IOUT
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xF83F), 31.5);       // TEMP_1
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xF847), 35.5);       // TEMP_2
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xF844), 34.0);       // TEMP_3
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x3827), 4992.0);     // FAN_1
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0xF252), 148.5);      // READ_PIN
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x0086), 134.0);      // READ_POUT
}

TEST(Linear11, ZeroWordDecodesToZero)
{
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x0000), 0.0);
}

TEST(Linear11, NegativeMantissa)
{
    // Mantissa 0x400 sign-extends to -1024, exponent 0.
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x0400), -1024.0);
}

TEST(Linear11, MaximumPositiveMantissaAndExponent)
{
    // Mantissa 0x3FF (1023), exponent 0x0F (15).
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x7BFF), 1023.0 * 32768.0);
}

TEST(Linear11, MaximumExponentWithNegativeMantissa)
{
    // Mantissa 0x7FF sign-extends to -1, exponent 0x0F (15).
    EXPECT_DOUBLE_EQ(psu::pmbus::decodeLinear11(0x7FFF), -32768.0);
}
