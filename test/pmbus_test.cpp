#include "pmbus.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

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

// ASCII: length-prefixed PMBus block (MFR_ID 0x99 / MFR_MODEL 0x9A).
TEST(PmbusAscii, DecodesManufacturerId)
{
    const std::vector<std::uint8_t> bytes = {5, 'A', 'C', 'B', 'E', 'L'};
    EXPECT_EQ(psu::pmbus::decodeAscii(bytes), "ACBEL");
}

TEST(PmbusAscii, DecodesModel)
{
    const std::vector<std::uint8_t> bytes = {
        14, 'R', '1', 'B', 'A', '2', '7', '5',
        '1', 'A', '-', 'P', '0', '0', 'A',
    };
    EXPECT_EQ(psu::pmbus::decodeAscii(bytes), "R1BA2751A-P00A");
}

TEST(PmbusAscii, ZeroLengthDecodesToEmpty)
{
    EXPECT_EQ(psu::pmbus::decodeAscii({0}), "");
}

TEST(PmbusAscii, RejectsEmptyBlock)
{
    EXPECT_THROW(psu::pmbus::decodeAscii({}), std::runtime_error);
}

TEST(PmbusAscii, RejectsTruncatedBlock)
{
    const std::vector<std::uint8_t> bytes = {5, 'A', 'B'};
    EXPECT_THROW(psu::pmbus::decodeAscii(bytes), std::runtime_error);
}

TEST(PmbusAscii, IgnoresTrailingBytes)
{
    const std::vector<std::uint8_t> bytes = {2, 'O', 'K', 0xFF};
    EXPECT_EQ(psu::pmbus::decodeAscii(bytes), "OK");
}

// STATUS_WORD (0x79): [6] OFF, [11] POWER_GOOD# (active-low: 0 indicates
// asserted/good, 1 indicates negated/not good).
TEST(PmbusStatusWord, DecodesProbedHostOffWord)
{
    // 0x0840 observed on the board in standby: OFF = 1, POWER_GOOD# = 1 (negated / not good).
    const auto status = psu::pmbus::decodeStatusWord(0x0840);
    EXPECT_TRUE(status.off);
    EXPECT_FALSE(status.powerGood);
}

TEST(PmbusStatusWord, DecodesAllClear)
{
    // 0x0000 in running state: OFF = 0, POWER_GOOD# = 0 (asserted / good).
    const auto status = psu::pmbus::decodeStatusWord(0x0000);
    EXPECT_FALSE(status.off);
    EXPECT_TRUE(status.powerGood);
}

TEST(PmbusStatusWord, DecodesOffWithPowerGoodAsserted)
{
    // 0x0040: OFF = 1, POWER_GOOD# = 0 (asserted / good).
    const auto status = psu::pmbus::decodeStatusWord(0x0040);
    EXPECT_TRUE(status.off);
    EXPECT_TRUE(status.powerGood);
}

TEST(PmbusStatusWord, DecodesPowerGoodNegatedWithoutOff)
{
    // 0x0800: output is on (OFF = 0), but POWER_GOOD# = 1 (negated / not good).
    const auto status = psu::pmbus::decodeStatusWord(0x0800);
    EXPECT_FALSE(status.off);
    EXPECT_FALSE(status.powerGood);
}

// Board cross-checks (#10): the decoded input/output power pairs are
// internally consistent (VIN*IIN ~ PIN, VOUT*IOUT ~ POUT) within 5%.
TEST(PmbusCrossCheck, InputPowerMatchesVinTimesIin)
{
    const double vin = psu::pmbus::decodeLinear11(0x007A);
    const double iin = psu::pmbus::decodeLinear11(0xD04B);
    const double pin = psu::pmbus::decodeLinear11(0xF252);
    EXPECT_NEAR(vin * iin, pin, pin * 0.05);
}

TEST(PmbusCrossCheck, OutputPowerMatchesVoutTimesIout)
{
    const double vout =
        psu::pmbus::decodeLinear16(0x00C3, psu::pmbus::voutModeExponent(0x1C));
    const double iout = psu::pmbus::decodeLinear11(0xE855);
    const double pout = psu::pmbus::decodeLinear11(0x0086);
    EXPECT_NEAR(vout * iout, pout, pout * 0.05);
}
