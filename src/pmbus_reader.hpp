#pragma once

#include <cstdint>
#include <functional>
#include <optional>

namespace psu::sensors
{

// One PMBus word: the raw 16-bit value, or no value when the read failed,
// timed out, or was malformed.
using WordResult = std::optional<std::uint16_t>;

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
