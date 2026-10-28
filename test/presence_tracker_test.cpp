#include "presence_tracker.hpp"

#include <gtest/gtest.h>

namespace
{

using psu::sensors::PresenceChange;
using psu::sensors::PresenceTracker;
using psu::sensors::ReadOutcome;

TEST(PresenceTrackerTest, Single0x80DebouncesWithoutUnplugging)
{
    PresenceTracker tracker(/*initiallyPresent=*/true, /*unplugThreshold=*/2);
    EXPECT_TRUE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 0U);

    // 1st cycle of 0x80 (device timeout)
    const auto change = tracker.record(ReadOutcome::deviceTimeout);
    EXPECT_EQ(change, PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 1U);
}

TEST(PresenceTrackerTest, TwoConsecutive0x80TransitionsToAbsent)
{
    PresenceTracker tracker(/*initiallyPresent=*/true, /*unplugThreshold=*/2);

    // 1st cycle
    EXPECT_EQ(tracker.record(ReadOutcome::deviceTimeout), PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());

    // 2nd consecutive cycle
    EXPECT_EQ(tracker.record(ReadOutcome::deviceTimeout),
              PresenceChange::removed);
    EXPECT_FALSE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 2U);
}

TEST(PresenceTrackerTest, 0xC0PreservesPresenceAndInterruptsTimeoutStreak)
{
    PresenceTracker tracker(/*initiallyPresent=*/true, /*unplugThreshold=*/2);

    // 1st cycle 0x80
    tracker.record(ReadOutcome::deviceTimeout);
    EXPECT_EQ(tracker.consecutiveTimeouts(), 1U);

    // 2nd cycle 0xC0 (ME Node Busy) preserves presence and resets streak
    const auto change1 = tracker.record(ReadOutcome::nodeBusy);
    EXPECT_EQ(change1, PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 0U);

    // Subsequent 0xC0 cycles keep presence true
    const auto change2 = tracker.record(ReadOutcome::nodeBusy);
    EXPECT_EQ(change2, PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());

    // Next 0x80 is only 1st consecutive again, so no unplug
    const auto change3 = tracker.record(ReadOutcome::deviceTimeout);
    EXPECT_EQ(change3, PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 1U);
}

TEST(PresenceTrackerTest, TransportErrorPreservesPresence)
{
    PresenceTracker tracker(/*initiallyPresent=*/true, /*unplugThreshold=*/2);

    for (int i = 0; i < 5; ++i)
    {
        EXPECT_EQ(tracker.record(ReadOutcome::transportError),
                  PresenceChange::none);
        EXPECT_TRUE(tracker.isPresent());
    }
}

TEST(PresenceTrackerTest, CleanReadRestoresPresenceImmediatelyOnAbsentSlot)
{
    PresenceTracker tracker(/*initiallyPresent=*/false, /*unplugThreshold=*/2);
    EXPECT_FALSE(tracker.isPresent());

    // 0x00 restores presence and yields PresenceChange::inserted
    const auto change = tracker.record(ReadOutcome::success);
    EXPECT_EQ(change, PresenceChange::inserted);
    EXPECT_TRUE(tracker.isPresent());
    EXPECT_EQ(tracker.consecutiveTimeouts(), 0U);

    // Subsequent clean read preserves presence without redundant inserted signal
    EXPECT_EQ(tracker.record(ReadOutcome::success), PresenceChange::none);
    EXPECT_TRUE(tracker.isPresent());
}

TEST(PresenceTrackerTest, AbsentSlotPreservesAbsenceOn0x80And0xC0)
{
    PresenceTracker tracker(/*initiallyPresent=*/false, /*unplugThreshold=*/2);

    EXPECT_EQ(tracker.record(ReadOutcome::deviceTimeout), PresenceChange::none);
    EXPECT_FALSE(tracker.isPresent());

    EXPECT_EQ(tracker.record(ReadOutcome::nodeBusy), PresenceChange::none);
    EXPECT_FALSE(tracker.isPresent());

    EXPECT_EQ(tracker.record(ReadOutcome::transportError),
              PresenceChange::none);
    EXPECT_FALSE(tracker.isPresent());
}

} // namespace
