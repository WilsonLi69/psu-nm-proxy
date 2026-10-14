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
    // One decoded value per sensor, std::nullopt where that read failed.
    std::vector<std::optional<double>> values;
    // Decoded STATUS_WORD, or std::nullopt when that read failed.
    std::optional<pmbus::StatusWord> status;
    // True when every sensor read succeeded.
    bool readsSucceeded = true;
};

// Drives one full read cycle asynchronously: each selected sensor's word read
// in turn, then STATUS_WORD, decoding as results arrive.
//
// It never blocks: run() returns immediately and the completion callback fires
// on the io_context thread once the whole cycle has finished. A failed read
// records std::nullopt and does not stop the cycle, so one slow or absent
// sensor never stalls the others or the D-Bus service.
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
        readSensor(0);
    }

  private:
    void readSensor(std::size_t index)
    {
        if (index == sensorDefs.size())
        {
            readStatus();
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
                    // Publish each value as soon as its read succeeds, rather
                    // than holding it until the whole cycle (including the
                    // slower STATUS_WORD) completes.
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

    void readStatus()
    {
        reader.read(statusWordCommand, [this](WordResult word) {
            if (word)
            {
                result.status = pmbus::decodeStatusWord(*word);
            }
            auto done = std::move(this->done);
            done(std::move(result));
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
