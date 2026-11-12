#include "fault_episode.hpp"

#include <gtest/gtest.h>

using psu::sensors::EpisodeTransition;
using psu::sensors::FaultEpisodeTracker;

namespace
{

// STATUS_WORD samples: the critical mask is 0x2704 (spec §5.4).
constexpr std::uint16_t vinUvFault = 0x2000;
constexpr std::uint16_t fanFault = 0x0400;
constexpr std::uint16_t ioutOcFault = 0x0200;
constexpr std::uint16_t clean = 0x0000;
constexpr std::uint16_t standbyClean = 0x0840; // OFF + POWER_GOOD#, no faults

} // namespace

TEST(FaultEpisodeTrackerTest, StartsClosedAndIgnoresCleanCycles)
{
    FaultEpisodeTracker episode;

    EXPECT_FALSE(episode.isOpen());
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(clean)),
              EpisodeTransition::none);
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(standbyClean)),
              EpisodeTransition::none);
    EXPECT_FALSE(episode.isOpen());
}

TEST(FaultEpisodeTrackerTest, OpensOnFirstCriticalFault)
{
    FaultEpisodeTracker episode;

    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);
    EXPECT_TRUE(episode.isOpen());
}

TEST(FaultEpisodeTrackerTest, StaysOpenWhileTheSameFaultPersists)
{
    FaultEpisodeTracker episode;
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);

    for (int cycle = 0; cycle < 5; ++cycle)
    {
        EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
                  EpisodeTransition::none);
        EXPECT_TRUE(episode.isOpen());
    }
}

TEST(FaultEpisodeTrackerTest, IgnoresBitChangesWhileOpen)
{
    FaultEpisodeTracker episode;
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);

    // An additional bit appearing, and the original bit clearing while another
    // remains asserted, are both still the same open episode.
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault |
                                                           ioutOcFault)),
              EpisodeTransition::none);
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(ioutOcFault)),
              EpisodeTransition::none);
    EXPECT_TRUE(episode.isOpen());
}

TEST(FaultEpisodeTrackerTest, ClosesOnTheFirstFaultFreeCycle)
{
    FaultEpisodeTracker episode;
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);

    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(clean)),
              EpisodeTransition::closed);
    EXPECT_FALSE(episode.isOpen());

    // The latch is re-armed: further clean cycles are silent.
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(clean)),
              EpisodeTransition::none);
}

TEST(FaultEpisodeTrackerTest, RemembersInputPowerLossAsTheOpeningCause)
{
    FaultEpisodeTracker episode;

    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(vinUvFault)),
              EpisodeTransition::opened);
    EXPECT_TRUE(episode.openedAsInputPowerLoss());
}

TEST(FaultEpisodeTrackerTest, NonInputFaultIsNotAnInputPowerLoss)
{
    FaultEpisodeTracker episode;

    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(ioutOcFault)),
              EpisodeTransition::opened);
    EXPECT_FALSE(episode.openedAsInputPowerLoss());
}

TEST(FaultEpisodeTrackerTest, OpeningCauseSurvivesLaterBitChanges)
{
    FaultEpisodeTracker episode;

    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(vinUvFault)),
              EpisodeTransition::opened);
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::none);

    // The episode opened on input-power loss even though VIN_UV_FAULT has
    // since cleared, so its recovery record must still be PowerRestored.
    EXPECT_TRUE(episode.openedAsInputPowerLoss());
}

TEST(FaultEpisodeTrackerTest, RemovalClosesTheEpisodeAndReArmsTheLatch)
{
    FaultEpisodeTracker episode;
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);

    EXPECT_TRUE(episode.close());
    EXPECT_FALSE(episode.isOpen());
    EXPECT_FALSE(episode.openedAsInputPowerLoss());

    // Closed on removal, so the slot is silent until a fault is observed again.
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(clean)),
              EpisodeTransition::none);
}

TEST(FaultEpisodeTrackerTest, ReinsertedFaultyUnitOpensANewEpisode)
{
    FaultEpisodeTracker episode;
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(vinUvFault)),
              EpisodeTransition::opened);
    ASSERT_TRUE(episode.close());

    // The unit returns still faulted: that is a new episode, announced again.
    EXPECT_EQ(episode.observe(psu::pmbus::decodeStatusWord(vinUvFault)),
              EpisodeTransition::opened);
    EXPECT_TRUE(episode.isOpen());
    EXPECT_TRUE(episode.openedAsInputPowerLoss());
}

TEST(FaultEpisodeTrackerTest, CloseReportsWhetherAnEpisodeWasOpen)
{
    FaultEpisodeTracker episode;

    EXPECT_FALSE(episode.close());
    ASSERT_EQ(episode.observe(psu::pmbus::decodeStatusWord(fanFault)),
              EpisodeTransition::opened);
    EXPECT_TRUE(episode.close());
}
