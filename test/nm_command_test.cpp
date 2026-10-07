#include "nm_command.hpp"

#include <gtest/gtest.h>

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
