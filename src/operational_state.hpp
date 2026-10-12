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
// follows only the consecutive-failure tracker; Functional additionally
// requires a successful STATUS_WORD read reporting POWER_GOOD; State follows
// the STATUS_WORD OFF bit. A successful read with POWER_GOOD=0 therefore keeps
// Present true and State Enabled.
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
    const bool functional =
        trackerOk && status.has_value() && status->powerGood;
    const bool disabled = status.has_value() && status->off;
    return {trackerOk, functional,
            disabled ? operationalStateDisabled : operationalStateEnabled};
}

} // namespace psu::sensors
