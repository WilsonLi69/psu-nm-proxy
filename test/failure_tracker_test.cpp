#include "failure_tracker.hpp"

#include <gtest/gtest.h>

using psu::sensors::FailureTracker;

TEST(FailureTracker, StartsFunctionalWithNoFailures)
{
    const FailureTracker tracker;

    EXPECT_TRUE(tracker.functional());
    EXPECT_EQ(tracker.consecutiveFailures(), 0U);
    EXPECT_EQ(tracker.threshold(), 3U);
}

TEST(FailureTracker, CountsConsecutiveFailures)
{
    FailureTracker tracker;

    tracker.recordFailure();
    EXPECT_EQ(tracker.consecutiveFailures(), 1U);
    EXPECT_TRUE(tracker.functional());

    tracker.recordFailure();
    EXPECT_EQ(tracker.consecutiveFailures(), 2U);
    EXPECT_TRUE(tracker.functional());
}

TEST(FailureTracker, StaysFunctionalBelowThreshold)
{
    FailureTracker tracker(3);

    tracker.recordFailure();
    tracker.recordFailure();

    EXPECT_TRUE(tracker.functional());
}

TEST(FailureTracker, BecomesNotFunctionalAtThreshold)
{
    FailureTracker tracker(3);

    tracker.recordFailure();
    tracker.recordFailure();
    tracker.recordFailure();

    EXPECT_FALSE(tracker.functional());
}

TEST(FailureTracker, StaysNotFunctionalBeyondThreshold)
{
    FailureTracker tracker(3);

    for (int i = 0; i < 5; i++)
    {
        tracker.recordFailure();
    }

    EXPECT_FALSE(tracker.functional());
}

TEST(FailureTracker, SuccessClearsFailuresAndRestoresFunctional)
{
    FailureTracker tracker(3);
    tracker.recordFailure();
    tracker.recordFailure();
    tracker.recordFailure();
    ASSERT_FALSE(tracker.functional());

    tracker.recordSuccess();

    EXPECT_TRUE(tracker.functional());
    EXPECT_EQ(tracker.consecutiveFailures(), 0U);
}

TEST(FailureTracker, SuccessBetweenFailuresResetsCounter)
{
    FailureTracker tracker(3);

    tracker.recordFailure();
    tracker.recordFailure();
    tracker.recordSuccess();
    EXPECT_EQ(tracker.consecutiveFailures(), 0U);

    tracker.recordFailure();
    tracker.recordFailure();
    EXPECT_TRUE(tracker.functional());

    tracker.recordFailure();
    EXPECT_FALSE(tracker.functional());
}

TEST(FailureTracker, ThresholdFromConstructor)
{
    FailureTracker one(1);
    one.recordFailure();
    EXPECT_FALSE(one.functional());

    FailureTracker five(5);
    for (int i = 0; i < 4; i++)
    {
        five.recordFailure();
    }
    EXPECT_TRUE(five.functional());

    five.recordFailure();
    EXPECT_FALSE(five.functional());
}

TEST(FailureTracker, SuccessOnFreshTrackerIsNoop)
{
    FailureTracker tracker;

    tracker.recordSuccess();

    EXPECT_TRUE(tracker.functional());
    EXPECT_EQ(tracker.consecutiveFailures(), 0U);
}
