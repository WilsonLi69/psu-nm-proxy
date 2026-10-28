#include "operational_state.hpp"
#include "pmbus.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{

using psu::pmbus::decodeStatusWord;
using psu::pmbus::StatusWord;
using psu::sensors::SlotHealth;
using psu::sensors::StateProfile;

} // namespace

// Standby host state S5 (OFF = 1, POWER_GOOD# = 1) and active running state S0
// (OFF = 0) both operate under STATE_OPERATIONAL without degrading Functional
// or Available (ADR-0003, spec §5.7).
TEST(OperationalState, RunningStateS0OperatesUnderStateOperational)
{
    const auto status = decodeStatusWord(0x0000);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, StandbyStateS5OperatesUnderStateOperational)
{
    // 0x0840 observed on the board in standby: OFF = 1, POWER_GOOD# = 1.
    const auto status = decodeStatusWord(0x0840);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, RunningStateWithPowerGoodNegatedOperatesUnderStateOperational)
{
    // 0x0800: output is on (OFF = 0), but POWER_GOOD# = 1 (negated). Output
    // status reflects output telemetry without degrading Functional or Available.
    const auto status = decodeStatusWord(0x0800);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

// CML (bit 1) and all WARNING bits in STATUS_WORD are ignored for health
// evaluation (ADR-0003, spec §5.7).
TEST(OperationalState, CmlBitIsIgnoredForHealthEvaluation)
{
    // CML error (bit 1: 0x0002) asserted in running state
    const auto status = decodeStatusWord(0x0002);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, WarningBitsAreIgnoredForHealthEvaluation)
{
    // Warning bits (bit 14: IOUT/POUT warning, bit 15: VOUT_OV warning -> 0xC000)
    const auto status = decodeStatusWord(0xC000);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, CombinedCmlAndWarningsDoNotDegradeHealth)
{
    // Standby S5 (0x0840) with CML and Warning bits set -> 0xC842
    const auto status = decodeStatusWord(0xC842);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_TRUE(health.available);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

// A present slot transitions to profile STATE_HW_FAULT (Present = true,
// Available = true, Functional = false) if any critical hardware fault bit in
// mask 0x2704 is asserted (ADR-0003, spec §5.7).
TEST(OperationalState, TemperatureFaultAppliesStateHwFault)
{
    // [2] TEMPERATURE_FAULT (0x0004)
    const auto status = decodeStatusWord(0x0004);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
    EXPECT_EQ(psu::sensors::firstCriticalFaultName(status), "TEMPERATURE_FAULT");
}

TEST(OperationalState, VoutOvFaultAppliesStateHwFault)
{
    // [8] VOUT_OV_FAULT (0x0100)
    const auto status = decodeStatusWord(0x0100);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
    EXPECT_EQ(psu::sensors::firstCriticalFaultName(status), "VOUT_OV_FAULT");
}

TEST(OperationalState, IoutOcFaultAppliesStateHwFault)
{
    // [9] IOUT_OC_FAULT (0x0200)
    const auto status = decodeStatusWord(0x0200);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
    EXPECT_EQ(psu::sensors::firstCriticalFaultName(status), "IOUT_OC_FAULT");
}

TEST(OperationalState, FanFaultAppliesStateHwFault)
{
    // [10] FAN_FAULT (0x0400)
    const auto status = decodeStatusWord(0x0400);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
    EXPECT_EQ(psu::sensors::firstCriticalFaultName(status), "FAN_FAULT");
}

TEST(OperationalState, VinUvFaultAppliesStateHwFault)
{
    // [13] VIN_UV_FAULT (0x2000)
    const auto status = decodeStatusWord(0x2000);
    const auto health = psu::sensors::slotHealth(true, status);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
    EXPECT_EQ(psu::sensors::firstCriticalFaultName(status), "VIN_UV_FAULT");
}

// A present slot transitions to profile STATE_COMM_FAULT (Present = true,
// Available = false, Functional = false) when consecutive communication
// failures reach threshold (trackerOk = false) (ADR-0003, spec §5.7).
TEST(OperationalState, SustainedCommsFailureAppliesStateCommFault)
{
    const auto health =
        psu::sensors::slotHealth(true, /*commsOk=*/false, std::nullopt, false);

    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::commFault);
}

// Transient communication failure (1st or 2nd consecutive failure, trackerOk = true)
// preserves the current profile without premature health degradation (ADR-0003).
TEST(OperationalState, TransientCommsFailurePreservesCurrentProfile)
{
    // Slot currently in STATE_OPERATIONAL: transient failure keeps STATE_OPERATIONAL
    const auto healthOp = psu::sensors::evaluateSlotHealth(
        true, /*commsOk=*/true, std::nullopt, false, StateProfile::operational);

    EXPECT_TRUE(healthOp.present);
    EXPECT_TRUE(healthOp.available);
    EXPECT_TRUE(healthOp.functional);
    EXPECT_EQ(healthOp.profile, StateProfile::operational);

    // Slot currently in STATE_HW_FAULT: transient failure keeps STATE_HW_FAULT
    const auto healthHw = psu::sensors::evaluateSlotHealth(
        true, /*commsOk=*/true, std::nullopt, false, StateProfile::hwFault);

    EXPECT_TRUE(healthHw.present);
    EXPECT_TRUE(healthHw.available);
    EXPECT_FALSE(healthHw.functional);
    EXPECT_EQ(healthHw.profile, StateProfile::hwFault);
}

// A valid poll (0x00) with STATUS_WORD & 0x2704 == 0 recovers a slot in
// STATE_COMM_FAULT or STATE_HW_FAULT back to STATE_OPERATIONAL (Present = true,
// Available = true, Functional = true) (ADR-0003, spec §5.7).
TEST(OperationalState, CleanPollRecoversFromStateCommFaultToStateOperational)
{
    const auto status = decodeStatusWord(0x0000);
    const auto health = psu::sensors::evaluateSlotHealth(
        true, /*commsOk=*/true, status, true, StateProfile::commFault);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, CleanPollRecoversFromStateHwFaultToStateOperational)
{
    const auto status = decodeStatusWord(0x0000);
    const auto health = psu::sensors::evaluateSlotHealth(
        true, /*commsOk=*/true, status, true, StateProfile::hwFault);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

// Resumed communication with an asserted critical hardware fault transitions
// from STATE_COMM_FAULT directly to STATE_HW_FAULT.
TEST(OperationalState, ValidPollWithCriticalFaultTransitionsFromCommFaultToHwFault)
{
    const auto status = decodeStatusWord(0x0400); // FAN_FAULT
    const auto health = psu::sensors::evaluateSlotHealth(
        true, /*commsOk=*/true, status, true, StateProfile::commFault);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
}

// If a slot is marked absent (currentlyPresent = false), completing a successful
// poll cycle restores Present = true and transitions to STATE_OPERATIONAL (or
// STATE_HW_FAULT if critical fault is asserted) (ADR-0003, spec §5.7).
TEST(OperationalState, AbsentSlotRestoresOperationalOnCleanRead)
{
    const auto status = decodeStatusWord(0x0000);
    const auto health = psu::sensors::evaluateSlotHealth(
        false, /*commsOk=*/true, status, true, StateProfile::absent);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::operational);
}

TEST(OperationalState, AbsentSlotRestoresHwFaultOnCleanReadWithFault)
{
    const auto status = decodeStatusWord(0x0100); // VOUT_OV_FAULT
    const auto health = psu::sensors::evaluateSlotHealth(
        false, /*commsOk=*/true, status, true, StateProfile::absent);

    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::hwFault);
}

TEST(OperationalState, AbsentSlotRemainsAbsentOnFailedCycle)
{
    const auto health = psu::sensors::evaluateSlotHealth(
        false, /*commsOk=*/true, std::nullopt, false, StateProfile::absent);

    EXPECT_FALSE(health.present);
    EXPECT_FALSE(health.available);
    EXPECT_FALSE(health.functional);
    EXPECT_EQ(health.profile, StateProfile::absent);
}

TEST(OperationalState, HealthEqualityAndProfileConstruction)
{
    EXPECT_EQ(psu::sensors::makeAbsentHealth(),
              (SlotHealth{false, false, false, StateProfile::absent}));
    EXPECT_EQ(psu::sensors::makeCommFaultHealth(),
              (SlotHealth{true, false, false, StateProfile::commFault}));
    EXPECT_EQ(psu::sensors::makeHwFaultHealth(),
              (SlotHealth{true, false, true, StateProfile::hwFault}));
    EXPECT_EQ(psu::sensors::makeOperationalHealth(),
              (SlotHealth{true, true, true, StateProfile::operational}));

    // Auto profile deduction from (present, functional, available) tuple
    EXPECT_EQ((SlotHealth{false, false, false}), psu::sensors::makeAbsentHealth());
    EXPECT_EQ((SlotHealth{true, false, false}), psu::sensors::makeCommFaultHealth());
    EXPECT_EQ((SlotHealth{true, false, true}), psu::sensors::makeHwFaultHealth());
    EXPECT_EQ((SlotHealth{true, true, true}), psu::sensors::makeOperationalHealth());
}
