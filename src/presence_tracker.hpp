#pragma once

#include "pmbus_reader.hpp"

#include <cstddef>

namespace psu::sensors
{

enum class PresenceChange
{
    none,
    inserted,
    removed,
};

// Tracks dynamic PSU presence over the Intel ME D9h proxy (spec §2.2/§5.3).
//
// Differentiates 0x80 (SMLink Device Timeout) from 0xC0 (ME Node Busy) and
// transport errors:
// - Two consecutive 0x80 cycles transition a present slot to absent (Present = false).
// - Single 0x80 is debounced (Present remains true).
// - 0xC0 (Node Busy) and IPMB transport errors preserve the current Present state,
//   counting only toward BMC<->ME communication health tracking.
// - A 0x00 (Success) cycle restores presence (Present = true) immediately.
class PresenceTracker
{
  public:
    explicit PresenceTracker(bool initiallyPresent = false,
                             std::size_t unplugThreshold = 2) :
        present(initiallyPresent),
        unplugThreshold(unplugThreshold)
    {}

    PresenceChange record(ReadOutcome outcome)
    {
        if (outcome == ReadOutcome::success)
        {
            consecutive0x80 = 0;
            if (!present)
            {
                present = true;
                return PresenceChange::inserted;
            }
            return PresenceChange::none;
        }

        if (outcome == ReadOutcome::deviceTimeout)
        {
            ++consecutive0x80;
            if (present && consecutive0x80 >= unplugThreshold)
            {
                present = false;
                return PresenceChange::removed;
            }
            return PresenceChange::none;
        }

        // On NodeBusy (0xC0) or TransportError:
        // Preserves Present state without penalty. Resets consecutive 0x80 counter
        // because SMLink timeout streak was interrupted.
        consecutive0x80 = 0;
        return PresenceChange::none;
    }

    bool isPresent() const noexcept
    {
        return present;
    }

    void setPresent(bool p) noexcept
    {
        present = p;
        consecutive0x80 = 0;
    }

    std::size_t consecutiveTimeouts() const noexcept
    {
        return consecutive0x80;
    }

    std::size_t threshold() const noexcept
    {
        return unplugThreshold;
    }

  private:
    bool present = false;
    std::size_t unplugThreshold = 2;
    std::size_t consecutive0x80 = 0;
};

} // namespace psu::sensors
