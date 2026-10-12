#pragma once

#include "failure_tracker.hpp"
#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "operational_state.hpp"
#include "pmbus.hpp"
#include "sensor_set.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace psu::sensors
{

inline constexpr auto sensorValueInterface = "xyz.openbmc_project.Sensor.Value";
inline constexpr auto operationalStatusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";

// Called whenever a slot's Present/Functional pair changes (spec §5.7):
// Present follows the consecutive-failure tracker, while Functional
// additionally requires a successful STATUS_WORD read reporting POWER_GOOD.
using FunctionalSink = std::function<void(bool present, bool functional)>;

// Publishes the §8 sensor set for one PSU and refreshes it every interval.
// Unconditional polling: never gated on host power state (spec §5.4).
class Poller
{
  public:
    Poller(boost::asio::io_context& io, sdbusplus::asio::connection& conn,
           sdbusplus::asio::object_server& server, std::string psuName,
           std::uint8_t psuAddr8, std::chrono::milliseconds interval,
           std::size_t failuresBeforeNotFunctional,
           std::vector<const SensorDef*> sensorDefs,
           std::map<std::string, Thresholds> thresholds = {},
           FunctionalSink onFunctionalChange = {}) :
        conn(conn), server(server), psuName(std::move(psuName)),
        psuAddr8(psuAddr8), interval(interval), timer(io),
        tracker(failuresBeforeNotFunctional), sensorDefs(std::move(sensorDefs)),
        thresholds(std::move(thresholds)),
        onFunctionalChange(std::move(onFunctionalChange))
    {
        addSensors();
        readVoutExponent();
    }

    void start()
    {
        poll();
    }

  private:
    static constexpr int defaultVoutExponent = -4;

    struct Published
    {
        const SensorDef* def;
        std::shared_ptr<sdbusplus::asio::dbus_interface> value;
        std::shared_ptr<sdbusplus::asio::dbus_interface> status;
    };

    void addSensors()
    {
        for (const auto* def : sensorDefs)
        {
            const std::string path = sensorPath(psuName, *def);

            auto value = server.add_interface(path, sensorValueInterface);
            value->register_property("Value", 0.0);
            value->register_property("Unit", unitType(def->unit));
            value->initialize();

            auto status =
                server.add_interface(path, operationalStatusInterface);
            status->register_property("Functional", true);
            status->register_property("State",
                                      std::string(operationalStateEnabled));
            status->initialize();

            addThresholds(path, def->label);

            sensors.push_back(
                Published{def, std::move(value), std::move(status)});
        }
    }

    // Register the configured threshold interfaces on the sensor's own path
    // (spec §5.6). A label with no configured thresholds publishes nothing.
    void addThresholds(const std::string& path, std::string_view label)
    {
        const auto it = thresholds.find(std::string(label));
        if (it == thresholds.end())
        {
            return;
        }

        const auto props = thresholdProperties(it->second);
        addThresholdInterface(path, warningThresholdInterface, props.warning);
        addThresholdInterface(path, criticalThresholdInterface,
                              props.critical);
    }

    void addThresholdInterface(const std::string& path,
                               const char* interfaceName,
                               const std::map<std::string, double>& properties)
    {
        if (properties.empty())
        {
            return;
        }
        auto iface = server.add_interface(path, interfaceName);
        for (const auto& [name, value] : properties)
        {
            iface->register_property(name, value);
        }
        iface->initialize();
    }

    std::uint16_t readWord(std::uint8_t command)
    {
        const auto payload = psu::nm::encodeReadWord(psuAddr8, command);
        const auto bytes = psu::ipmb::sendRawPmbus(conn, payload);
        return psu::nm::wordFromLittleEndian(bytes);
    }

    void readVoutExponent()
    {
        try
        {
            voutExponent = psu::pmbus::voutModeExponent(
                static_cast<std::uint8_t>(readWord(voutModeCommand)));
        }
        catch (const std::exception&)
        {
            voutExponent = defaultVoutExponent;
        }
    }

    double decode(const SensorDef& def, std::uint16_t word) const
    {
        if (def.format == Format::linear16)
        {
            return psu::pmbus::decodeLinear16(word, voutExponent);
        }
        return psu::pmbus::decodeLinear11(word);
    }

    void poll()
    {
        bool readsSucceeded = true;

        for (auto& sensor : sensors)
        {
            try
            {
                const auto word = readWord(sensor.def->command);
                sensor.value->set_property("Value",
                                           decode(*sensor.def, word));
            }
            catch (const std::exception&)
            {
                readsSucceeded = false;
            }
        }

        // A cycle is a failure when any D9h read failed; a clean cycle clears
        // the consecutive-failure count (spec §5.7).
        const auto statusWord = readStatusWord();
        const bool cycleOk = readsSucceeded && statusWord.has_value();
        if (cycleOk)
        {
            tracker.recordSuccess();
        }
        else
        {
            tracker.recordFailure();
        }

        const auto health = slotHealth(tracker.functional(), statusWord);
        for (auto& sensor : sensors)
        {
            sensor.status->set_property("Functional", health.functional);
            sensor.status->set_property("State", std::string(health.state));
        }
        publishFailureTransition(health.present, health.functional);

        timer.expires_after(interval);
        timer.async_wait([this](const boost::system::error_code&) { poll(); });
    }

    // Returns the decoded STATUS_WORD, or no value when the read itself failed.
    std::optional<psu::pmbus::StatusWord> readStatusWord()
    {
        try
        {
            return psu::pmbus::decodeStatusWord(readWord(statusWordCommand));
        }
        catch (const std::exception&)
        {
            return std::nullopt;
        }
    }

    // Emit the enabling/disabling transition once per crossing and mirror the
    // slot's Present/Functional pair onto inventory (spec §5.7).
    void publishFailureTransition(bool present, bool functional)
    {
        const bool trackerChanged = present != lastPresent;
        if (!trackerChanged && functional == lastFunctional)
        {
            return;
        }
        lastPresent = present;
        lastFunctional = functional;

        if (onFunctionalChange)
        {
            onFunctionalChange(present, functional);
        }

        if (trackerChanged)
        {
            if (present)
            {
                std::cerr << psuName << ": functional again\n";
            }
            else
            {
                std::cerr << psuName << ": not functional after "
                          << tracker.threshold() << " consecutive failures\n";
            }
        }
    }

    sdbusplus::asio::connection& conn;
    sdbusplus::asio::object_server& server;
    std::string psuName;
    std::uint8_t psuAddr8;
    std::chrono::milliseconds interval;
    boost::asio::steady_timer timer;
    std::vector<Published> sensors;
    int voutExponent = defaultVoutExponent;
    FailureTracker tracker;
    std::vector<const SensorDef*> sensorDefs;
    std::map<std::string, Thresholds> thresholds;
    FunctionalSink onFunctionalChange;
    bool lastPresent = true;
    bool lastFunctional = true;
};

} // namespace psu::sensors
