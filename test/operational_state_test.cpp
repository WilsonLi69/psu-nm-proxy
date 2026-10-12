#include "operational_state.hpp"
#include "pmbus.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{

using psu::pmbus::StatusWord;
using psu::sensors::SlotHealth;

} // namespace

// A successful STATUS_WORD read with OFF=0 / POWER_GOOD=1 is present,
// functional and enabled (spec §5.7).
TEST(OperationalState, EnabledWhenOutputOn)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{false, true});

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// OFF=1 -> output disabled; POWER_GOOD drives Functional (spec §5.7).
TEST(OperationalState, DisabledWhenOffBitSet)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{true, false});

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateDisabled);
}

// A successful read with POWER_GOOD=0 must not clear Present: only the
// consecutive-failure tracker moves Present (spec §5.6/§5.7).
TEST(OperationalState, PowerGoodFalseKeepsPresentAndEnabled)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{false, false});

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// Below the failure threshold the tracker is still "ok", but a missing
// STATUS_WORD read leaves Functional false (spec §5.7).
TEST(OperationalState, MissingStatusIsNotFunctional)
{
    const auto health = psu::sensors::slotHealth(true, std::nullopt);

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
}

// Crossing the consecutive-failure threshold clears Present as well as
// Functional (spec §5.7).
TEST(OperationalState, TrackerFailureClearsPresentAndFunctional)
{
    const auto health = psu::sensors::slotHealth(false, StatusWord{false, true});

    EXPECT_FALSE(health.present);
    EXPECT_FALSE(health.functional);
}

TEST(OperationalState, HealthEquality)
{
    EXPECT_EQ(psu::sensors::slotHealth(true, StatusWord{false, true}),
              (SlotHealth{true, true, psu::sensors::operationalStateEnabled}));
}
