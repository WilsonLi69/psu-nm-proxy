#pragma once

#include <cstddef>

namespace psu::sensors
{

// Tracks consecutive D9h (raw PMBus) read failures for one PSU slot. A slot
// stays functional until `threshold` failures occur back-to-back; any success
// clears the count and restores functionality (spec §5.7).
class FailureTracker
{
  public:
    explicit FailureTracker(std::size_t threshold = 3) :
        thresholdValue(threshold)
    {}

    void recordSuccess()
    {
        failureCount = 0;
    }

    void recordFailure()
    {
        ++failureCount;
    }

    bool functional() const
    {
        return failureCount < thresholdValue;
    }

    std::size_t consecutiveFailures() const
    {
        return failureCount;
    }

    std::size_t threshold() const
    {
        return thresholdValue;
    }

  private:
    std::size_t thresholdValue;
    std::size_t failureCount = 0;
};

} // namespace psu::sensors
