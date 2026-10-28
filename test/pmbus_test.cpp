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
// asserted/good, 1 indicates negated/not good), and the 5 critical hardware
// fault bits in mask 0x2704 ([2] TEMPERATURE_FAULT, [8] VOUT_OV_FAULT,
// [9] IOUT_OC_FAULT, [10] FAN_FAULT, [13] VIN_UV_FAULT).
TEST(PmbusStatusWord, DecodesProbedHostOffWord)
{
    // 0x0840 observed on the board in standby: OFF = 1, POWER_GOOD# = 1 (negated / not good).
    const auto status = psu::pmbus::decodeStatusWord(0x0840);
    EXPECT_EQ(status.raw, 0x0840);
    EXPECT_TRUE(status.off);
    EXPECT_FALSE(status.powerGood);
    EXPECT_FALSE(status.hasCriticalFault());
    EXPECT_FALSE(status.temperatureFault);
    EXPECT_FALSE(status.voutOvFault);
    EXPECT_FALSE(status.ioutOcFault);
    EXPECT_FALSE(status.fanFault);
    EXPECT_FALSE(status.vinUvFault);
}

TEST(PmbusStatusWord, DecodesAllClear)
{
    // 0x0000 in running state: OFF = 0, POWER_GOOD# = 0 (asserted / good).
    const auto status = psu::pmbus::decodeStatusWord(0x0000);
    EXPECT_EQ(status.raw, 0x0000);
    EXPECT_FALSE(status.off);
    EXPECT_TRUE(status.powerGood);
    EXPECT_FALSE(status.hasCriticalFault());
}

TEST(PmbusStatusWord, DecodesOffWithPowerGoodAsserted)
{
    // 0x0040: OFF = 1, POWER_GOOD# = 0 (asserted / good).
    const auto status = psu::pmbus::decodeStatusWord(0x0040);
    EXPECT_EQ(status.raw, 0x0040);
    EXPECT_TRUE(status.off);
    EXPECT_TRUE(status.powerGood);
    EXPECT_FALSE(status.hasCriticalFault());
}

TEST(PmbusStatusWord, DecodesPowerGoodNegatedWithoutOff)
{
    // 0x0800: output is on (OFF = 0), but POWER_GOOD# = 1 (negated / not good).
    const auto status = psu::pmbus::decodeStatusWord(0x0800);
    EXPECT_EQ(status.raw, 0x0800);
    EXPECT_FALSE(status.off);
    EXPECT_FALSE(status.powerGood);
    EXPECT_FALSE(status.hasCriticalFault());
}

TEST(PmbusStatusWord, DecodesCriticalFaultBitsIndividually)
{
    // [2] TEMPERATURE_FAULT (0x0004)
    const auto tempStatus = psu::pmbus::decodeStatusWord(0x0004);
    EXPECT_TRUE(tempStatus.temperatureFault);
    EXPECT_FALSE(tempStatus.voutOvFault);
    EXPECT_FALSE(tempStatus.ioutOcFault);
    EXPECT_FALSE(tempStatus.fanFault);
    EXPECT_FALSE(tempStatus.vinUvFault);
    EXPECT_TRUE(tempStatus.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(tempStatus), "TEMPERATURE_FAULT");
    EXPECT_EQ(psu::pmbus::criticalFaultName(0x0004), "TEMPERATURE_FAULT");

    // [8] VOUT_OV_FAULT (0x0100)
    const auto voutStatus = psu::pmbus::decodeStatusWord(0x0100);
    EXPECT_FALSE(voutStatus.temperatureFault);
    EXPECT_TRUE(voutStatus.voutOvFault);
    EXPECT_FALSE(voutStatus.ioutOcFault);
    EXPECT_FALSE(voutStatus.fanFault);
    EXPECT_FALSE(voutStatus.vinUvFault);
    EXPECT_TRUE(voutStatus.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(voutStatus), "VOUT_OV_FAULT");

    // [9] IOUT_OC_FAULT (0x0200)
    const auto ioutStatus = psu::pmbus::decodeStatusWord(0x0200);
    EXPECT_FALSE(ioutStatus.temperatureFault);
    EXPECT_FALSE(ioutStatus.voutOvFault);
    EXPECT_TRUE(ioutStatus.ioutOcFault);
    EXPECT_FALSE(ioutStatus.fanFault);
    EXPECT_FALSE(ioutStatus.vinUvFault);
    EXPECT_TRUE(ioutStatus.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(ioutStatus), "IOUT_OC_FAULT");

    // [10] FAN_FAULT (0x0400)
    const auto fanStatus = psu::pmbus::decodeStatusWord(0x0400);
    EXPECT_FALSE(fanStatus.temperatureFault);
    EXPECT_FALSE(fanStatus.voutOvFault);
    EXPECT_FALSE(fanStatus.ioutOcFault);
    EXPECT_TRUE(fanStatus.fanFault);
    EXPECT_FALSE(fanStatus.vinUvFault);
    EXPECT_TRUE(fanStatus.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(fanStatus), "FAN_FAULT");

    // [13] VIN_UV_FAULT (0x2000)
    const auto vinStatus = psu::pmbus::decodeStatusWord(0x2000);
    EXPECT_FALSE(vinStatus.temperatureFault);
    EXPECT_FALSE(vinStatus.voutOvFault);
    EXPECT_FALSE(vinStatus.ioutOcFault);
    EXPECT_FALSE(vinStatus.fanFault);
    EXPECT_TRUE(vinStatus.vinUvFault);
    EXPECT_TRUE(vinStatus.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(vinStatus), "VIN_UV_FAULT");
}

TEST(PmbusStatusWord, DecodesCombinedCriticalFaultMask0x2704)
{
    const auto status = psu::pmbus::decodeStatusWord(0x2704);
    EXPECT_TRUE(status.temperatureFault);
    EXPECT_TRUE(status.voutOvFault);
    EXPECT_TRUE(status.ioutOcFault);
    EXPECT_TRUE(status.fanFault);
    EXPECT_TRUE(status.vinUvFault);
    EXPECT_TRUE(status.hasCriticalFault());
    EXPECT_TRUE(psu::pmbus::hasCriticalFault(0x2704));
    // firstCriticalFaultName returns the first asserted bit in the mask
    EXPECT_EQ(psu::pmbus::firstCriticalFaultName(status), "TEMPERATURE_FAULT");
}

TEST(PmbusStatusWord, IgnoresCmlAndWarningBitsInCriticalFaultEvaluation)
{
    // CML (bit 1: 0x0002)
    const auto cmlStatus = psu::pmbus::decodeStatusWord(0x0002);
    EXPECT_FALSE(cmlStatus.hasCriticalFault());
    EXPECT_FALSE(psu::pmbus::hasCriticalFault(0x0002));
    EXPECT_EQ(psu::pmbus::criticalFaultName(cmlStatus), "");

    // WARNING bits: bit 14 (IOUT/POUT warning: 0x4000), bit 15 (0x8000)
    const auto warnStatus = psu::pmbus::decodeStatusWord(0xC000);
    EXPECT_FALSE(warnStatus.hasCriticalFault());
    EXPECT_FALSE(psu::pmbus::hasCriticalFault(0xC000));
    EXPECT_EQ(psu::pmbus::criticalFaultName(warnStatus), "");

    // Host standby S5 (0x0840) with CML and Warning bits set (0xC842)
    const auto standbyWithNoise = psu::pmbus::decodeStatusWord(0xC842);
    EXPECT_FALSE(standbyWithNoise.hasCriticalFault());
    EXPECT_EQ(psu::pmbus::criticalFaultName(standbyWithNoise), "");
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
