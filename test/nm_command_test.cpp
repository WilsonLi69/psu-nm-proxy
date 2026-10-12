#include "nm_command.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

// Observed hardware payload from docs/research/10-acbel-psu-command-set.md:
// sendRequest yyyyay 0 0x2e 0 0xd9 10 <ten-byte payload ending in the command>
TEST(NmCommand, EncodesReadVinForSlotTwo)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x06, 0xB2, 0x00, 0x00, 0x01, 0x02, 0x88,
    };
    EXPECT_EQ(psu::nm::encodeReadWord(0xB2, 0x88), expected);
}

TEST(NmCommand, EncodesReadWordForSlotOne)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x06, 0xB0, 0x00, 0x00, 0x01, 0x02, 0x8B,
    };
    EXPECT_EQ(psu::nm::encodeReadWord(0xB0, 0x8B), expected);
}

// MFR_ID/MFR_MODEL are PMBus BLOCK_READs: flags 0x0a, write length 0x01 and a
// caller-supplied read length. Observed board probe:
//   ... 0a <addr8> 00 00 01 10 99
TEST(NmCommand, EncodesBlockReadForMfrId)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x0A, 0xB2, 0x00, 0x00, 0x01, 0x10, 0x99,
    };
    EXPECT_EQ(psu::nm::encodeBlockRead(0xB2, 0x99, 0x10), expected);
}

TEST(NmCommand, EncodesBlockReadForMfrModel)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x0A, 0xB2, 0x00, 0x00, 0x01, 0x10, 0x9A,
    };
    EXPECT_EQ(psu::nm::encodeBlockRead(0xB2, 0x9A, 0x10), expected);
}

TEST(NmCommand, EncodesBlockReadWithVariableReadLength)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x0A, 0xB0, 0x00, 0x00, 0x01, 0x20, 0x9A,
    };
    EXPECT_EQ(psu::nm::encodeBlockRead(0xB0, 0x9A, 0x20), expected);
}

TEST(NmCommand, DetectsIntelManufacturerEcho)
{
    EXPECT_TRUE(psu::nm::hasIntelEcho({0x87, 0x01, 0x00, 0x05, 0x41}));
    EXPECT_FALSE(psu::nm::hasIntelEcho({0x57, 0x01, 0x00, 0x05}));
    EXPECT_FALSE(psu::nm::hasIntelEcho({0x87, 0x01}));
    EXPECT_FALSE(psu::nm::hasIntelEcho({}));
}

TEST(NmCommand, StripsIntelManufacturerEcho)
{
    const std::vector<std::uint8_t> response = {0x87, 0x01, 0x00, 0x7A, 0x00};
    const std::vector<std::uint8_t> expected = {0x7A, 0x00};
    EXPECT_EQ(psu::nm::stripIntelEcho(response), expected);
}

TEST(NmCommand, RejectsResponseWithoutEcho)
{
    const std::vector<std::uint8_t> response = {0x87, 0x01, 0x7A, 0x00};
    EXPECT_THROW(psu::nm::stripIntelEcho(response), std::runtime_error);
}

TEST(NmCommand, RejectsShortResponse)
{
    const std::vector<std::uint8_t> response = {0x87, 0x01};
    EXPECT_THROW(psu::nm::stripIntelEcho(response), std::runtime_error);
}

TEST(NmCommand, AssemblesLittleEndianWord)
{
    const std::vector<std::uint8_t> bytes = {0x7A, 0x00};
    EXPECT_EQ(psu::nm::wordFromLittleEndian(bytes), 0x007A);

    const std::vector<std::uint8_t> offset = {0x00, 0x4B, 0xD0};
    EXPECT_EQ(psu::nm::wordFromLittleEndian(offset, 1), 0xD04B);
}

TEST(NmCommand, RejectsShortWord)
{
    EXPECT_THROW(psu::nm::wordFromLittleEndian({0x7A}), std::runtime_error);
}

TEST(NmCommand, EncodesGetFwCapabilities)
{
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x20, 0x00, 0x3F,
    };
    EXPECT_EQ(psu::nm::encodeGetFwCapabilities(), expected);
}

// Observed DEh response after echo strip:
// 04 44 50 00 02 88 00 01 0F 03, capability byte idx 8 = 0x0F.
TEST(NmCommand, AcceptsPmbusProxy11b)
{
    const std::vector<std::uint8_t> payload = {
        0x04, 0x44, 0x50, 0x00, 0x02, 0x88, 0x00, 0x01, 0x0F, 0x03,
    };
    EXPECT_TRUE(psu::nm::pmbusProxyEnabled(payload));
}

TEST(NmCommand, AcceptsPmbusProxy01b)
{
    const std::vector<std::uint8_t> payload = {
        0x04, 0x44, 0x50, 0x00, 0x02, 0x88, 0x00, 0x01, 0x04, 0x03,
    };
    EXPECT_TRUE(psu::nm::pmbusProxyEnabled(payload));
}

TEST(NmCommand, RejectsPmbusProxy00b)
{
    const std::vector<std::uint8_t> payload = {
        0x04, 0x44, 0x50, 0x00, 0x02, 0x88, 0x00, 0x01, 0x00, 0x03,
    };
    EXPECT_FALSE(psu::nm::pmbusProxyEnabled(payload));
}

TEST(NmCommand, RejectsPmbusProxy10b)
{
    const std::vector<std::uint8_t> payload = {
        0x04, 0x44, 0x50, 0x00, 0x02, 0x88, 0x00, 0x01, 0x08, 0x03,
    };
    EXPECT_FALSE(psu::nm::pmbusProxyEnabled(payload));
}

TEST(NmCommand, RejectsShortCapabilityPayload)
{
    EXPECT_FALSE(psu::nm::pmbusProxyEnabled({}));
    EXPECT_FALSE(psu::nm::pmbusProxyEnabled(
        {0x04, 0x44, 0x50, 0x00, 0x02, 0x88, 0x00, 0x01}));
}

TEST(NmCommand, EncodesGetPsuConfig)
{
    EXPECT_EQ(psu::nm::encodeGetPsuConfig(), (std::vector<std::uint8_t>{
        0x57, 0x01, 0x00, 0x00}));
    EXPECT_EQ(psu::nm::encodeGetPsuConfig(3), (std::vector<std::uint8_t>{
        0x57, 0x01, 0x00, 0x03}));
}

// Observed D8h response after echo strip:
// 00 B1 B3 00 00 00 00 00 00 -> slot 1 at 0xB0, slot 2 at 0xB2.
TEST(NmCommand, ParsesConfiguredSlots)
{
    const std::vector<std::uint8_t> payload = {
        0x00, 0xB1, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const std::vector<psu::nm::PsuSlot> expected = {
        {1, 0xB0},
        {2, 0xB2},
    };
    EXPECT_EQ(psu::nm::psuSlots(payload), expected);
    EXPECT_EQ(psu::nm::psuAddresses8(payload),
              (std::vector<std::uint8_t>{0xB0, 0xB2}));
}

TEST(NmCommand, IgnoresUnconfiguredAndUnfilledSlots)
{
    EXPECT_TRUE(psu::nm::psuSlots({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00})
                    .empty());

    const std::vector<std::uint8_t> gap = {
        0x00, 0x00, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    EXPECT_EQ(psu::nm::psuSlots(gap),
              (std::vector<psu::nm::PsuSlot>{{2, 0xB2}}));
}

TEST(NmCommand, MasksPsuModeBit)
{
    const std::vector<std::uint8_t> payload = {
        0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    EXPECT_EQ(psu::nm::psuAddresses8(payload),
              (std::vector<std::uint8_t>{0xFE}));
}

TEST(NmCommand, EncodesSetPsuConfig)
{
    const std::array<std::uint8_t, 8> addrs = {0xB0, 0xB2, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00};
    const std::vector<std::uint8_t> expected = {
        0x57, 0x01, 0x00, 0x00, 0xB0, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    EXPECT_EQ(psu::nm::encodeSetPsuConfig(0, addrs), expected);
}

TEST(NmCommand, SetPsuConfigFallbackOffByDefault)
{
    const std::array<std::uint8_t, 8> addrs = {0xB0, 0xB2, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00};
    const std::vector<std::uint8_t> noSlots = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    EXPECT_FALSE(psu::nm::setPsuConfigFallback(false, noSlots, 0, addrs)
                     .has_value());
}

TEST(NmCommand, SetPsuConfigFallbackTrustsD8h)
{
    const std::array<std::uint8_t, 8> addrs = {0xB0, 0xB2, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00};
    const std::vector<std::uint8_t> configured = {
        0x00, 0xB1, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    EXPECT_FALSE(psu::nm::setPsuConfigFallback(true, configured, 0, addrs)
                     .has_value());
}

TEST(NmCommand, SetPsuConfigFallbackUsedOnlyWhenEnabledAndEmpty)
{
    const std::array<std::uint8_t, 8> addrs = {0xB0, 0xB2, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00};
    const std::vector<std::uint8_t> noSlots = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const auto payload = psu::nm::setPsuConfigFallback(true, noSlots, 0, addrs);
    ASSERT_TRUE(payload.has_value());
    EXPECT_EQ(*payload, psu::nm::encodeSetPsuConfig(0, addrs));
}
