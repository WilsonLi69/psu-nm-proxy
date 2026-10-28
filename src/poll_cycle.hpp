#pragma once

#include "pmbus.hpp"
#include "pmbus_reader.hpp"
#include "sensor_set.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace psu::sensors
{

// The decoded outcome of one asynchronous read cycle for a single slot.
struct CycleResult
{
    // Typed outcome of the STATUS_WORD read (or cycle outcome).
    ReadOutcome outcome = ReadOutcome::success;
    // One decoded value per sensor, std::nullopt where that read failed.
    std::vector<std::optional<double>> values;
    // Decoded STATUS_WORD, or std::nullopt when that read failed.
    std::optional<pmbus::StatusWord> status;
    // True when every sensor read succeeded.
    bool readsSucceeded = true;
};

// Drives one full read cycle asynchronously: STATUS_WORD (0x79) is read first.
// If 0x80 (DeviceTimeout) or 0xC0 (NodeBusy) is returned, the remaining telemetry
// reads for that cycle are short-circuited (spec §5.3). Otherwise, each selected
// sensor is read in turn.
//
// It never blocks: run() returns immediately and the completion callback fires
// on the io_context thread once the whole cycle has finished.
//
// One cycle at a time: run() must not be called again until `done` has fired.
class PollCycle
{
  public:
    PollCycle(WordReader& reader, std::vector<const SensorDef*> sensorDefs) :
        reader(reader), sensorDefs(std::move(sensorDefs))
    {}

    void run(int voutExponent, std::function<void(std::size_t, double)> onValue,
             std::function<void(CycleResult)> done)
    {
        this->voutExponent = voutExponent;
        this->onValue = std::move(onValue);
        this->done = std::move(done);
        result = CycleResult{};
        result.values.assign(sensorDefs.size(), std::nullopt);
        readStatusWord();
    }

  private:
    void readStatusWord()
    {
        reader.read(statusWordCommand, [this](WordResult word) {
            result.outcome = word.outcome;

            // Short-circuit remaining telemetry reads on SMLink device timeout (0x80)
            // or ME node busy (0xC0).
            if (word.outcome == ReadOutcome::deviceTimeout ||
                word.outcome == ReadOutcome::nodeBusy)
            {
                result.readsSucceeded = false;
                auto done = std::move(this->done);
                done(std::move(result));
                return;
            }

            if (word.outcome != ReadOutcome::success || !word.has_value())
            {
                result.readsSucceeded = false;
                if (result.outcome == ReadOutcome::success)
                {
                    result.outcome = ReadOutcome::transportError;
                }
                auto done = std::move(this->done);
                done(std::move(result));
                return;
            }

            try
            {
                result.status = pmbus::decodeStatusWord(*word);
            }
            catch (const std::exception&)
            {
                result.readsSucceeded = false;
                result.outcome = ReadOutcome::transportError;
                auto done = std::move(this->done);
                done(std::move(result));
                return;
            }

            // STATUS_WORD read succeeded; read configured sensors.
            readSensor(0);
        });
    }

    void readSensor(std::size_t index)
    {
        if (index == sensorDefs.size())
        {
            auto done = std::move(this->done);
            done(std::move(result));
            return;
        }

        const SensorDef* def = sensorDefs[index];
        reader.read(def->command, [this, index, def](WordResult word) {
            if (!word)
            {
                result.readsSucceeded = false;
            }
            else
            {
                try
                {
                    const double value = decode(*def, *word);
                    result.values[index] = value;
                    if (onValue)
                    {
                        onValue(index, value);
                    }
                }
                catch (const std::exception&)
                {
                    result.readsSucceeded = false;
                }
            }
            readSensor(index + 1);
        });
    }

    double decode(const SensorDef& def, std::uint16_t word) const
    {
        if (def.format == Format::linear16)
        {
            return pmbus::decodeLinear16(word, voutExponent);
        }
        return pmbus::decodeLinear11(word);
    }

    WordReader& reader;
    std::vector<const SensorDef*> sensorDefs;
    int voutExponent = -4;
    CycleResult result;
    std::function<void(std::size_t, double)> onValue;
    std::function<void(CycleResult)> done;
};

} // namespace psu::sensors
