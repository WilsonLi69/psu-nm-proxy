#include "operational_state.hpp"
#include "pmbus.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{

using psu::pmbus::StatusWord;
using psu::sensors::SlotHealth;

} // namespace

// A successful STATUS_WORD read with OFF=0 / powerGood=true (POWER_GOOD#=0)
// is present, functional and enabled (spec §5.7).
TEST(OperationalState, EnabledWhenOutputOn)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{false, true});

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// In standby with OFF=1 (output disabled, POWER_GOOD#=1 -> powerGood=false),
// the slot remains functional with State=Disabled (spec §5.7).
TEST(OperationalState, DisabledWhenOffBitSet)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{true, false});

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateDisabled);
}

// When output is enabled (OFF=0) but power is negated (powerGood=false,
// POWER_GOOD#=1), Functional degrades to false while keeping Present true
// and State Enabled (spec §5.6/§5.7).
TEST(OperationalState, PowerGoodFalseKeepsPresentAndEnabled)
{
    const auto health = psu::sensors::slotHealth(true, StatusWord{false, false});

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// Verify end-to-end integration with decodeStatusWord() for host-off standby (0x0840).
TEST(OperationalState, StandbyPowerStateFromDecodedStatusWord)
{
    const auto status = psu::pmbus::decodeStatusWord(0x0840);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateDisabled);
}

// Verify end-to-end integration with decodeStatusWord() for host-on active running (0x0000).
TEST(OperationalState, RunningPowerStateFromDecodedStatusWord)
{
    const auto status = psu::pmbus::decodeStatusWord(0x0000);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
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

// Crossing the consecutive-failure threshold clears Functional while
// maintaining Present=true for an installed PSU (spec §5.7).
TEST(OperationalState, TrackerFailureClearsFunctionalWhileMaintainingPresent)
{
    const auto health = psu::sensors::slotHealth(false, StatusWord{false, true});

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
}

// For an installed PSU (currentlyPresent=true), consecutive communication
// failure preserves Present=true while clearing Functional (spec §5.7).
TEST(OperationalState, InstalledSlotPreservesPresentOnCommunicationFailure)
{
    const auto health =
        psu::sensors::slotHealth(true, false, std::nullopt, false);

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
}

// If a slot is marked absent (currentlyPresent=false), completing a successful
// poll cycle (readsSucceeded && status.has_value()) restores Present=true and
// Functional=true (spec §5.7).
TEST(OperationalState, AbsentSlotRestoresPresentAndFunctionalOnCleanRead)
{
    const auto health = psu::sensors::slotHealth(
        false, true, StatusWord{false, true}, true);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// If a slot is marked absent (currentlyPresent=false), a failed cycle
// keeps it absent and non-functional (spec §5.7).
TEST(OperationalState, AbsentSlotRemainsAbsentOnFailedCycle)
{
    const auto health =
        psu::sensors::slotHealth(false, true, std::nullopt, false);

    EXPECT_FALSE(health.present);
    EXPECT_FALSE(health.functional);
}

// If a slot is marked absent (currentlyPresent=false), a cycle where sensor reads
// fail despite status having a value does not restore presence (spec §5.7).
TEST(OperationalState, AbsentSlotRemainsAbsentWhenReadsFailDespiteStatus)
{
    const auto health = psu::sensors::slotHealth(
        false, true, StatusWord{false, true}, false);

    EXPECT_FALSE(health.present);
    EXPECT_FALSE(health.functional);
}

TEST(OperationalState, HealthEquality)
{
    EXPECT_EQ(psu::sensors::slotHealth(true, StatusWord{false, true}),
              (SlotHealth{true, true, psu::sensors::operationalStateEnabled}));
    EXPECT_EQ(psu::sensors::slotHealth(false, StatusWord{false, true}),
              (SlotHealth{true, false, psu::sensors::operationalStateEnabled}));
    EXPECT_EQ(psu::sensors::slotHealth(false, true, StatusWord{false, true}, true),
              (SlotHealth{true, true, psu::sensors::operationalStateEnabled}));
}
