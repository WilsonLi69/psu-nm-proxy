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
// follows only the consecutive-failure tracker; Functional is true when the
// slot communicates cleanly without tracker failure and the output status is
// healthy (in standby with output OFF, or in running state with POWER_GOOD
// asserted); State follows the STATUS_WORD OFF bit (Disabled when off, Enabled
// when running).
struct SlotHealth
{
    bool present;
    bool functional;
    std::string_view state;

    bool operator==(const SlotHealth&) const = default;
};

inline SlotHealth
    slotHealth(bool trackerOk,
               const std::optional<pmbus::StatusWord>& status)
{
    const bool disabled = status.has_value() && status->off;
    const bool functional =
        trackerOk && status.has_value() && (status->off || status->powerGood);
    return {trackerOk, functional,
            disabled ? operationalStateDisabled : operationalStateEnabled};
}

} // namespace psu::sensors
