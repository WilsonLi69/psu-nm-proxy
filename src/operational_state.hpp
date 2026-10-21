#pragma once

#include "pmbus.hpp"

#include <optional>
#include <string_view>

namespace psu::sensors
{

// xyz.openbmc_project.State.Decorator.OperationalStatus.State enum values
// (spec §5.7).
inline constexpr std::string_view operationalStateEnabled =
    "xyz.openbmc_project.State.Decorator.OperationalStatus.State.Enabled";
inline constexpr std::string_view operationalStateDisabled =
    "xyz.openbmc_project.State.Decorator.OperationalStatus.State.Disabled";

// The health published for one slot on a poll cycle (spec §5.6/§5.7). Present
// is decoupled from transient communication failures: for an installed PSU,
// consecutive communication failures degrade Functional to false but do not
// flip Present to false. If a slot is marked absent (Present=false), the first
// clean read cycle (readsSucceeded && status.has_value()) restores Present to
// true and Functional to true. Functional is true when the slot is present,
// the consecutive-failure tracker is functional, the cycle reads succeeded,
// and the output status is healthy (in standby with output OFF, or in running
// state with POWER_GOOD asserted); State follows the STATUS_WORD OFF bit
// (Disabled when off, Enabled when running).
struct SlotHealth
{
    bool present;
    bool functional;
    std::string_view state;

    bool operator==(const SlotHealth&) const = default;
};

inline SlotHealth
    slotHealth(bool currentlyPresent, bool trackerOk,
               const std::optional<pmbus::StatusWord>& status,
               bool readsSucceeded = true)
{
    const bool cleanCycle = readsSucceeded && status.has_value();
    const bool present = currentlyPresent || cleanCycle;
    const bool disabled = !present || (status.has_value() && status->off);
    const bool functional =
        present && trackerOk && cleanCycle && (status->off || status->powerGood);
    return {present, functional,
            disabled ? operationalStateDisabled : operationalStateEnabled};
}

inline SlotHealth
    slotHealth(bool trackerOk,
               const std::optional<pmbus::StatusWord>& status)
{
    return slotHealth(true, trackerOk, status, true);
}


} // namespace psu::sensors
