#pragma once

#include <cstdint>
#include <functional>
#include <optional>

namespace psu::sensors
{

// Standard read outcomes for PMBus transactions over IPMB (spec §2.1/§5.3).
enum class ReadOutcome : std::uint8_t
{
    success = 0x00,
    deviceTimeout = 0x80,
    nodeBusy = 0xC0,
    transportError = 0xFF,
};

// One PMBus word result: surfaces the typed ReadOutcome and the decoded 16-bit
// value (if successful).
struct WordResult
{
    ReadOutcome outcome = ReadOutcome::transportError;
    std::optional<std::uint16_t> value = std::nullopt;

    constexpr bool has_value() const noexcept
    {
        return value.has_value();
    }

    constexpr explicit operator bool() const noexcept
    {
        return outcome == ReadOutcome::success && value.has_value();
    }

    constexpr std::uint16_t operator*() const
    {
        return *value;
    }

    constexpr WordResult() = default;
    constexpr WordResult(ReadOutcome outcome) : outcome(outcome) {}
    constexpr WordResult(std::uint16_t val) :
        outcome(ReadOutcome::success), value(val)
    {}
    constexpr WordResult(ReadOutcome outcome, std::uint16_t val) :
        outcome(outcome), value(val)
    {}
    constexpr WordResult(ReadOutcome outcome, std::optional<std::uint16_t> val) :
        outcome(outcome), value(val)
    {}
    constexpr WordResult(std::nullopt_t) :
        outcome(ReadOutcome::transportError), value(std::nullopt)
    {}

    bool operator==(const WordResult&) const = default;
};

// Reads one PMBus command word from a single PSU through the ME D9h proxy.
//
// read() must return promptly and deliver its result later, on the io_context
// thread; it must never block the calling thread on the slow ME/IPMB. Keeping
// reads asynchronous is what leaves the io_context free to serve D-Bus
// requests while a poll is in flight.
class WordReader
{
  public:
    virtual ~WordReader() = default;

    virtual void read(std::uint8_t command,
                      std::function<void(WordResult)> callback) = 0;
};

} // namespace psu::sensors
