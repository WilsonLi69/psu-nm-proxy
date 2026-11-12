#pragma once

#include <cstdint>
#include <functional>
#include <optional>

namespace psu::sensors
{

// Standard read outcomes for PMBus transactions over IPMB (spec §2.1/§5.3).
// The values of the first three mirror the completion codes they name; the
// remaining two are internal tags, not codes the ME can return.
enum class ReadOutcome : std::uint16_t
{
    success = 0x00,
    deviceTimeout = 0x80,
    nodeBusy = 0xC0,
    // The ME or the PSU returned some other non-success completion code. The
    // code itself travels in WordResult::completionCode, so this outcome says
    // only "a code we do not classify further" (spec §5.2.3).
    unknownCompletionCode = 0x100,
    transportError = 0x101,
};

// One PMBus word result: surfaces the typed ReadOutcome and the decoded 16-bit
// value (if successful).
struct WordResult
{
    ReadOutcome outcome = ReadOutcome::transportError;
    std::optional<std::uint16_t> value = std::nullopt;
    // The completion code the ME or PSU returned, kept verbatim so a
    // communication-fault record can report it (spec §5.7.3). It is set for
    // every non-success outcome except transportError, where the D-Bus/IPMB
    // call failed and no code came back; that case is logged as a transport
    // timeout. A successful read carries no code either.
    std::optional<std::uint8_t> completionCode = std::nullopt;

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
    constexpr WordResult(ReadOutcome outcome) :
        outcome(outcome), completionCode(impliedCompletionCode(outcome))
    {}
    constexpr WordResult(std::uint16_t val) :
        outcome(ReadOutcome::success), value(val)
    {}
    constexpr WordResult(ReadOutcome outcome, std::uint16_t val) :
        outcome(outcome), value(val),
        completionCode(impliedCompletionCode(outcome))
    {}
    constexpr WordResult(ReadOutcome outcome, std::optional<std::uint16_t> val) :
        outcome(outcome), value(val),
        completionCode(impliedCompletionCode(outcome))
    {}
    constexpr WordResult(std::nullopt_t) :
        outcome(ReadOutcome::transportError), value(std::nullopt)
    {}

    // Builds the result of a transaction that returned `completionCode`, which
    // must be a non-success code: 0x80 and 0xC0 keep their own outcomes, every
    // other code becomes unknownCompletionCode while staying reportable.
    static constexpr WordResult fromCompletionCode(std::uint8_t completionCode)
    {
        WordResult result;
        result.completionCode = completionCode;
        if (completionCode == deviceTimeoutCode)
        {
            result.outcome = ReadOutcome::deviceTimeout;
        }
        else if (completionCode == nodeBusyCode)
        {
            result.outcome = ReadOutcome::nodeBusy;
        }
        else
        {
            result.outcome = ReadOutcome::unknownCompletionCode;
        }
        return result;
    }

    bool operator==(const WordResult&) const = default;

  private:
    static constexpr std::uint8_t deviceTimeoutCode =
        static_cast<std::uint8_t>(ReadOutcome::deviceTimeout);
    static constexpr std::uint8_t nodeBusyCode =
        static_cast<std::uint8_t>(ReadOutcome::nodeBusy);

    // The inverse of fromCompletionCode's classification: 0x80 and 0xC0 are
    // completion codes in their own right, so an outcome naming one already
    // knows the code the ME returned.
    static constexpr std::optional<std::uint8_t> impliedCompletionCode(
        ReadOutcome outcome)
    {
        switch (outcome)
        {
            case ReadOutcome::deviceTimeout:
                return deviceTimeoutCode;
            case ReadOutcome::nodeBusy:
                return nodeBusyCode;
            default:
                return std::nullopt;
        }
    }
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
