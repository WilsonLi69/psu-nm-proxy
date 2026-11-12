#pragma once

#include "pmbus.hpp"

namespace psu::sensors
{

enum class EpisodeTransition
{
    none,
    opened,
    closed,
};

// Latches a PSU hardware fault into an episode (spec §5.7.1, ADR-0004 §4).
//
// A hardware fault is an episode, not a poll event: the first cycle observing
// any critical bit in mask 0x2704 opens it, further faulted cycles change
// nothing even when the asserted bit set changes, and the first fault-free
// cycle closes it. Only successful poll cycles are fed to the latch, so
// communication faults never interrupt an episode and a resumed fault is not
// announced twice.
class FaultEpisodeTracker
{
  public:
    // Feed one completed, communicating cycle.
    EpisodeTransition observe(const pmbus::StatusWord& status)
    {
        if (!status.hasCriticalFault())
        {
            if (!open)
            {
                return EpisodeTransition::none;
            }
            open = false;
            inputPowerLoss = false;
            return EpisodeTransition::closed;
        }

        if (open)
        {
            return EpisodeTransition::none;
        }
        open = true;
        inputPowerLoss = pmbus::isInputPowerLoss(
            pmbus::firstCriticalFaultName(status));
        return EpisodeTransition::opened;
    }

    // Terminates an episode without a recovery record and re-arms the latch, so
    // a re-inserted unit that is still faulty opens a new episode (spec §5.7.1
    // rule 4). Returns whether an episode was open.
    bool close()
    {
        const bool wasOpen = open;
        open = false;
        inputPowerLoss = false;
        return wasOpen;
    }

    bool isOpen() const
    {
        return open;
    }

    // Whether the episode opened as loss of AC input, which decides between the
    // PowerSupplyPowerRestored and PowerSupplyRecovered recovery records.
    bool openedAsInputPowerLoss() const
    {
        return inputPowerLoss;
    }

  private:
    bool open = false;
    bool inputPowerLoss = false;
};

} // namespace psu::sensors
