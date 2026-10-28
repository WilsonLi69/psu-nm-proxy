#pragma once

#include "pmbus.hpp"

#include <optional>
#include <string_view>

namespace psu::sensors
{

using psu::pmbus::criticalFaultName;
using psu::pmbus::firstCriticalFaultName;
using psu::pmbus::hasCriticalFault;

// Canonical Composite State Profiles (ADR-0003 & spec §5.6/§5.7).
enum class StateProfile
{
    absent,
    commFault,
    hwFault,
    operational,
};

inline constexpr auto STATE_ABSENT = StateProfile::absent;
inline constexpr auto STATE_COMM_FAULT = StateProfile::commFault;
inline constexpr auto STATE_HW_FAULT = StateProfile::hwFault;
inline constexpr auto STATE_OPERATIONAL = StateProfile::operational;

inline constexpr std::string_view stateProfileName(StateProfile profile)
{
    switch (profile)
    {
        case StateProfile::absent:
            return "STATE_ABSENT";
        case StateProfile::commFault:
            return "STATE_COMM_FAULT";
        case StateProfile::hwFault:
            return "STATE_HW_FAULT";
        case StateProfile::operational:
            return "STATE_OPERATIONAL";
    }
    return "UNKNOWN";
}

// Composite functional health and presence state for one slot on a poll cycle
// (ADR-0003, spec §5.6/§5.7).
//
// D-Bus property mapping across phosphor-dbus-interfaces:
// - xyz.openbmc_project.Inventory.Item: Present (bool)
// - xyz.openbmc_project.State.Decorator.OperationalStatus: Functional (bool)
// - xyz.openbmc_project.State.Decorator.Availability: Available (bool)
//
// Profiles:
// - STATE_ABSENT:      Present = false, Available = false, Functional = false
// - STATE_COMM_FAULT:  Present = true,  Available = false, Functional = false
// - STATE_HW_FAULT:    Present = true,  Available = true,  Functional = false
// - STATE_OPERATIONAL: Present = true,  Available = true,  Functional = true
//
// Host standby state S5 (OFF=1, POWER_GOOD#=1) and active running state S0 (OFF=0)
// both operate under STATE_OPERATIONAL without degrading Functional or Available.
// Standby output rail status reflects output telemetry and does not degrade Functional.
// CML and all WARNING bits in STATUS_WORD are ignored for health evaluation.
struct SlotHealth
{
    bool present = false;
    bool functional = false;
    bool available = false;
    StateProfile profile = StateProfile::absent;

    SlotHealth() = default;

    SlotHealth(bool present, bool functional, bool available,
               StateProfile profile) :
        present(present), functional(functional), available(available),
        profile(profile)
    {}

    SlotHealth(bool present, bool functional, bool available) :
        present(present), functional(functional), available(available),
        profile(profileFromTuple(present, functional, available))
    {}

    bool operator==(const SlotHealth&) const = default;

  private:
    static constexpr StateProfile profileFromTuple(bool present, bool functional,
                                                   bool available)
    {
        if (!present)
        {
            return StateProfile::absent;
        }
        if (!functional)
        {
            return available ? StateProfile::hwFault : StateProfile::commFault;
        }
        return StateProfile::operational;
    }
};

inline constexpr SlotHealth makeAbsentHealth()
{
    return {false, false, false, StateProfile::absent};
}

inline constexpr SlotHealth makeCommFaultHealth()
{
    return {true, false, false, StateProfile::commFault};
}

inline constexpr SlotHealth makeHwFaultHealth()
{
    return {true, false, true, StateProfile::hwFault};
}

inline constexpr SlotHealth makeOperationalHealth()
{
    return {true, true, true, StateProfile::operational};
}

inline SlotHealth evaluateSlotHealth(
    bool currentlyPresent, bool commsOk,
    const std::optional<pmbus::StatusWord>& status,
    bool readsSucceeded = true,
    StateProfile currentProfile = StateProfile::operational)
{
    const bool cleanCycle = readsSucceeded && status.has_value();

    // Absent slot evaluation:
    if (!currentlyPresent)
    {
        if (!cleanCycle)
        {
            return makeAbsentHealth();
        }
        // First clean read on an absent slot restores presence
        if (status->hasCriticalFault())
        {
            return makeHwFaultHealth();
        }
        return makeOperationalHealth();
    }

    // Present slot evaluation:
    // If consecutive communication failures reach failuresBeforeNotFunctional, apply STATE_COMM_FAULT
    if (!commsOk)
    {
        return makeCommFaultHealth();
    }

    // Comms is OK:
    if (cleanCycle)
    {
        // Valid poll: check critical hardware faults (mask 0x2704)
        if (status->hasCriticalFault())
        {
            return makeHwFaultHealth();
        }
        // Clean read without critical hardware fault (including S0 running and S5 standby,
        // and with CML / WARNING bits ignored) -> STATE_OPERATIONAL
        return makeOperationalHealth();
    }

    // Comms is OK (1st or 2nd transient failure), but this cycle was not clean:
    // Retain the current profile (transient bus noise tolerance)
    switch (currentProfile)
    {
        case StateProfile::hwFault:
            return makeHwFaultHealth();
        case StateProfile::commFault:
            return makeCommFaultHealth();
        case StateProfile::absent:
            return makeAbsentHealth();
        case StateProfile::operational:
        default:
            return makeOperationalHealth();
    }
}

inline SlotHealth slotHealth(
    bool currentlyPresent, bool commsOk,
    const std::optional<pmbus::StatusWord>& status,
    bool readsSucceeded = true,
    StateProfile currentProfile = StateProfile::operational)
{
    return evaluateSlotHealth(currentlyPresent, commsOk, status, readsSucceeded,
                              currentProfile);
}

inline SlotHealth slotHealth(
    bool commsOk,
    const std::optional<pmbus::StatusWord>& status)
{
    return evaluateSlotHealth(true, commsOk, status, status.has_value(),
                              StateProfile::operational);
}

} // namespace psu::sensors
