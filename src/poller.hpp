#pragma once

#include "failure_tracker.hpp"
#include "ipmb_reader.hpp"
#include "operational_state.hpp"
#include "pmbus.hpp"
#include "poll_cycle.hpp"
#include "sensor_set.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace psu::sensors
{

inline constexpr auto sensorValueInterface = "xyz.openbmc_project.Sensor.Value";
inline constexpr auto operationalStatusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";

// Called whenever a slot's Present/Functional pair changes (spec §5.7):
// Present is decoupled from transient communication failures and restored on
// a clean read cycle, while Functional degrades on consecutive failures or
// power faults and is restored on successful read.
using FunctionalSink = std::function<void(bool present, bool functional)>;

// Publishes the §8 sensor set for one PSU and refreshes it every interval.
// Unconditional polling: never gated on host power state (spec §5.4).
//
// All Ipmb reads are asynchronous (see PollCycle / IpmbWordReader), so a poll
// never blocks the io_context that serves the object server: introspection and
// property reads stay answerable even while the ME is slow or absent.
class Poller
{
  public:
    Poller(boost::asio::io_context& io, sdbusplus::asio::connection& conn,
           sdbusplus::asio::object_server& server, std::string psuName,
           std::uint8_t psuAddr8, std::chrono::milliseconds interval,
           std::size_t failuresBeforeNotFunctional,
           std::vector<const SensorDef*> sensorDefs,
           std::map<std::string, Thresholds> thresholds = {},
           FunctionalSink onFunctionalChange = {},
           bool initiallyPresent = true) :
        server(server), psuName(std::move(psuName)), interval(interval),
        timer(io), tracker(failuresBeforeNotFunctional),
        reader(conn, psuAddr8), cycle(reader, sensorDefs),
        thresholds(std::move(thresholds)),
        onFunctionalChange(std::move(onFunctionalChange)),
        lastPresent(initiallyPresent),
        lastFunctional(initiallyPresent)
    {
        addSensors(sensorDefs);
    }

    void setPresent(bool present)
    {
        lastPresent = present;
        if (!present)
        {
            lastFunctional = false;
        }
    }

    bool isPresent() const
    {
        return lastPresent;
    }

    bool isFunctional() const
    {
        return lastFunctional;
    }


    void start()
    {
        // Read VOUT_MODE once, asynchronously, before the first cycle so
        // Linear16 uses the PSU's exponent; a failed read keeps the default.
        reader.read(voutModeCommand, [this](WordResult word) {
            if (word)
            {
                voutExponent =
                    pmbus::voutModeExponent(static_cast<std::uint8_t>(*word));
            }
            poll();
        });
    }

  private:
    static constexpr int defaultVoutExponent = -4;

    struct Published
    {
        const SensorDef* def;
        std::shared_ptr<sdbusplus::asio::dbus_interface> value;
        std::shared_ptr<sdbusplus::asio::dbus_interface> status;
    };

    void addSensors(const std::vector<const SensorDef*>& defs)
    {
        for (const auto* def : defs)
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

    void poll()
    {
        // Sensor values are published as each read succeeds (the onValue
        // sink); applyCycle then commits the cycle's health once it finishes.
        cycle.run(
            voutExponent,
            [this](std::size_t index, double value) {
                if (index < sensors.size())
                {
                    sensors[index].value->set_property("Value", value);
                }
            },
            [this](CycleResult result) {
                applyCycle(result);

                timer.expires_after(interval);
                timer.async_wait(
                    [this](const boost::system::error_code& ec) {
                        if (!ec)
                        {
                            poll();
                        }
                    });
            });
    }

    // Commit one completed cycle: advance the consecutive-failure tracker and
    // flip Present/Functional (spec §5.7). Values were already published as
    // their reads completed.
    void applyCycle(const CycleResult& result)
    {
        // A cycle is a failure when any D9h read failed; a clean cycle clears
        // the consecutive-failure count (spec §5.7).
        const bool cycleOk = result.readsSucceeded && result.status.has_value();
        if (cycleOk)
        {
            tracker.recordSuccess();
        }
        else
        {
            tracker.recordFailure();
        }

        const auto health =
            slotHealth(lastPresent, tracker.functional(), result.status,
                       result.readsSucceeded);
        for (auto& sensor : sensors)
        {
            sensor.status->set_property("Functional", health.functional);
            sensor.status->set_property("State", std::string(health.state));
        }
        publishFailureTransition(health.present, health.functional);
    }

    // Emit transitions once per crossing and mirror the slot's
    // Present/Functional pair onto inventory (spec §5.7).
    void publishFailureTransition(bool present, bool functional)
    {
        const bool presenceChanged = (present != lastPresent);
        const bool functionalChanged = (functional != lastFunctional);
        if (!presenceChanged && !functionalChanged)
        {
            return;
        }
        lastPresent = present;
        lastFunctional = functional;

        if (onFunctionalChange)
        {
            onFunctionalChange(present, functional);
        }

        if (presenceChanged)
        {
            if (present)
            {
                std::cerr << psuName << ": present again\n";
            }
            else
            {
                std::cerr << psuName << ": no longer present\n";
            }
        }

        if (functionalChanged)
        {
            if (functional)
            {
                std::cerr << psuName << ": functional again\n";
            }
            else
            {
                if (!tracker.functional())
                {
                    std::cerr << psuName << ": not functional after "
                              << tracker.threshold() << " consecutive failures\n";
                }
                else
                {
                    std::cerr << psuName << ": not functional\n";
                }
            }
        }
    }

    sdbusplus::asio::object_server& server;
    std::string psuName;
    std::chrono::milliseconds interval;
    boost::asio::steady_timer timer;
    std::vector<Published> sensors;
    int voutExponent = defaultVoutExponent;
    FailureTracker tracker;
    IpmbWordReader reader;
    PollCycle cycle;
    std::map<std::string, Thresholds> thresholds;
    FunctionalSink onFunctionalChange;
    bool lastPresent = true;
    bool lastFunctional = true;
};

} // namespace psu::sensors
