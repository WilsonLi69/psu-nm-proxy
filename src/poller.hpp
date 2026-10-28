#pragma once

#include "dbus_adapters.hpp"
#include "event_logger.hpp"
#include "failure_tracker.hpp"
#include "inventory.hpp"
#include "inventory_sink.hpp"
#include "ipmb_reader.hpp"
#include "node_manager_client.hpp"
#include "operational_state.hpp"
#include "pmbus.hpp"
#include "poll_cycle.hpp"
#include "presence_tracker.hpp"
#include "sensor_set.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
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
inline constexpr auto availabilityInterface =
    "xyz.openbmc_project.State.Decorator.Availability";

// Called whenever a slot's Present/Functional/Available health changes (spec §5.7).
// Supports both 2-argument (present, functional) and 3-argument
// (present, functional, available) callbacks for backward and forward compatibility.
class FunctionalSink
{
  public:
    FunctionalSink() = default;

    FunctionalSink(std::function<void(bool, bool, std::optional<bool>)> f) :
        fn3(std::move(f))
    {}

    FunctionalSink(std::function<void(bool, bool)> f) :
        fn2(std::move(f))
    {}

    template <typename Callable>
        requires std::invocable<Callable, bool, bool, std::optional<bool>>
    FunctionalSink(Callable&& c) :
        fn3(std::forward<Callable>(c))
    {}

    template <typename Callable>
        requires std::invocable<Callable, bool, bool> &&
                 (!std::invocable<Callable, bool, bool, std::optional<bool>>)
    FunctionalSink(Callable&& c) :
        fn2(std::forward<Callable>(c))
    {}

    void operator()(bool present, bool functional,
                    std::optional<bool> available = std::nullopt) const
    {
        if (fn3)
        {
            fn3(present, functional, available);
        }
        else if (fn2)
        {
            fn2(present, functional);
        }
    }

    explicit operator bool() const noexcept
    {
        return static_cast<bool>(fn3) || static_cast<bool>(fn2);
    }

  private:
    std::function<void(bool, bool, std::optional<bool>)> fn3;
    std::function<void(bool, bool)> fn2;
};

// Abstract sink for publishing and updating per-slot D-Bus sensor objects
// (Sensor.Value, Availability, OperationalStatus, and Thresholds). Allows Poller
// to be tested without a live sdbusplus::asio::object_server.
class SensorPublisher
{
  public:
    virtual ~SensorPublisher() = default;

    virtual void publishSensor(std::size_t index, const std::string& path,
                               const SensorDef& def, double initialValue,
                               bool initialFunctional, bool initialAvailable,
                               const ThresholdProperties& thresholds) = 0;

    virtual void updateValue(std::size_t index, double value) = 0;

    virtual void updateStatus(std::size_t index, bool functional,
                              bool available) = 0;
};

class DbusSensorPublisher : public SensorPublisher
{
  public:
    explicit DbusSensorPublisher(sdbusplus::asio::object_server& server,
                                 std::string inventoryPath = {}) :
        server(server), inventoryPath(std::move(inventoryPath))
    {}

    void publishSensor(std::size_t index, const std::string& path,
                       const SensorDef& def, double initialValue,
                       bool initialFunctional, bool initialAvailable,
                       const ThresholdProperties& thresholds) override
    {
        if (index >= sensors.size())
        {
            sensors.resize(index + 1);
        }

        auto value = server.add_interface(path, sensorValueInterface);
        value->register_property("Value", initialValue);
        value->register_property("Unit", unitType(def.unit));
        value->initialize();

        auto avail = server.add_interface(path, availabilityInterface);
        avail->register_property("Available", initialAvailable);
        avail->initialize();

        auto status = server.add_interface(path, operationalStatusInterface);
        status->register_property("Functional", initialFunctional);
        status->initialize();

        if (!inventoryPath.empty())
        {
            auto assoc = server.add_interface(
                path, psu::inventory::associationInterface);
            assoc->register_property(
                "Associations",
                std::vector<psu::inventory::Association>{
                    psu::inventory::sensorInventoryAssociation(inventoryPath)});
            assoc->initialize();
        }

        addThresholdInterface(path, warningThresholdInterface,
                               thresholds.warning);
        addThresholdInterface(path, criticalThresholdInterface,
                               thresholds.critical);

        sensors[index] = Published{std::move(value), std::move(status),
                                   std::move(avail)};
    }

    void updateValue(std::size_t index, double value) override
    {
        if (index < sensors.size() && sensors[index].value)
        {
            sensors[index].value->set_property("Value", value);
        }
    }

    void updateStatus(std::size_t index, bool functional,
                      bool available) override
    {
        if (index < sensors.size())
        {
            if (sensors[index].status)
            {
                sensors[index].status->set_property("Functional", functional);
            }
            if (sensors[index].availability)
            {
                sensors[index].availability->set_property("Available", available);
            }
        }
    }

  private:
    struct Published
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> value;
        std::shared_ptr<sdbusplus::asio::dbus_interface> status;
        std::shared_ptr<sdbusplus::asio::dbus_interface> availability;
    };

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

    sdbusplus::asio::object_server& server;
    std::string inventoryPath;
    std::vector<Published> sensors;
};

// Publishes the §8 sensor set for one PSU and refreshes it every interval.
// Unconditional polling: never gated on host power state (spec §5.4).
//
// All Ipmb reads are asynchronous (see PollCycle / IpmbWordReader), so a poll
// never blocks the io_context that serves the object server: introspection and
// property reads stay answerable even while the ME is slow or absent.
class Poller
{
  public:
    inline static psu::event::NullEventLogger& getNullLogger()
    {
        static psu::event::NullEventLogger instance;
        return instance;
    }

    Poller(boost::asio::io_context& io, sdbusplus::asio::connection& conn,
           sdbusplus::asio::object_server& server, std::string psuName,
           std::uint8_t psuAddr8, std::chrono::milliseconds interval,
           std::size_t failuresBeforeNotFunctional,
           std::vector<const SensorDef*> sensorDefs,
           std::map<std::string, Thresholds> thresholds = {},
           FunctionalSink onFunctionalChange = {},
           bool initiallyPresent = false,
           psu::discovery::NodeManagerClient* nmClient = nullptr,
           psu::discovery::InventorySink* invSink = nullptr,
           psu::event::EventLogger* eventLogger = nullptr) :
        ownedReader(std::make_unique<IpmbWordReader>(conn, psuAddr8)),
        ownedPublisher(std::make_unique<DbusSensorPublisher>(
            server, psu::inventory::powerSupplyPath(psuName))),
        ownedLogger(eventLogger ? nullptr
                                : std::make_unique<psu::discovery::DbusEventLogger>(conn)),
        psuName(std::move(psuName)), psuAddr8(psuAddr8), interval(interval),
        timer(io), sensorCount(sensorDefs.size()),
        presenceTracker(initiallyPresent),
        tracker(failuresBeforeNotFunctional),
        reader(*ownedReader), publisher(*ownedPublisher),
        logger(eventLogger ? *eventLogger : *ownedLogger),
        cycle(reader, sensorDefs), thresholds(std::move(thresholds)),
        onFunctionalChange(std::move(onFunctionalChange)),
        lastPresent(initiallyPresent), lastFunctional(initiallyPresent),
        lastAvailable(initiallyPresent),
        lastProfile(initiallyPresent ? StateProfile::operational
                                     : StateProfile::absent),
        nmClient(nmClient), invSink(invSink)
    {
        addSensors(sensorDefs);
    }

    Poller(boost::asio::io_context& io, WordReader& reader,
           SensorPublisher& publisher, std::string psuName,
           std::chrono::milliseconds interval,
           std::size_t failuresBeforeNotFunctional,
           std::vector<const SensorDef*> sensorDefs,
           std::map<std::string, Thresholds> thresholds = {},
           FunctionalSink onFunctionalChange = {},
           bool initiallyPresent = false,
           std::uint8_t psuAddr8 = 0,
           psu::discovery::NodeManagerClient* nmClient = nullptr,
           psu::discovery::InventorySink* invSink = nullptr,
           psu::event::EventLogger* eventLogger = nullptr) :
        psuName(std::move(psuName)), psuAddr8(psuAddr8), interval(interval),
        timer(io), sensorCount(sensorDefs.size()),
        presenceTracker(initiallyPresent),
        tracker(failuresBeforeNotFunctional),
        reader(reader), publisher(publisher),
        logger(eventLogger ? *eventLogger : getNullLogger()),
        cycle(reader, sensorDefs),
        thresholds(std::move(thresholds)),
        onFunctionalChange(std::move(onFunctionalChange)),
        lastPresent(initiallyPresent), lastFunctional(initiallyPresent),
        lastAvailable(initiallyPresent),
        lastProfile(initiallyPresent ? StateProfile::operational
                                     : StateProfile::absent),
        nmClient(nmClient), invSink(invSink)
    {
        addSensors(sensorDefs);
    }

    void setPresent(bool present)
    {
        presenceTracker.setPresent(present);
        lastPresent = present;
        if (!present)
        {
            lastFunctional = false;
            lastAvailable = false;
            lastProfile = StateProfile::absent;
            setAllSensorsNaN();
        }
    }

    bool isPresent() const
    {
        return presenceTracker.isPresent();
    }

    bool isFunctional() const
    {
        return lastFunctional;
    }

    bool isAvailable() const
    {
        return lastAvailable;
    }

    StateProfile getProfile() const
    {
        return lastProfile;
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

    void poll()
    {
        // Sensor values are published as each read succeeds (the onValue
        // sink); applyCycle then commits the cycle's health once it finishes.
        cycle.run(
            voutExponent,
            [this](std::size_t index, double value) {
                publisher.updateValue(index, value);
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

  private:
    static constexpr int defaultVoutExponent = -4;

    void addSensors(const std::vector<const SensorDef*>& defs)
    {
        const auto init = initialSensorState(lastPresent);
        for (std::size_t i = 0; i < defs.size(); ++i)
        {
            const auto* def = defs[i];
            const std::string path = sensorPath(psuName, *def);

            ThresholdProperties threshProps;
            const auto it = thresholds.find(std::string(def->label));
            if (it != thresholds.end())
            {
                threshProps = thresholdProperties(it->second);
            }

            publisher.publishSensor(i, path, *def, init.value, init.functional,
                                    init.available, threshProps);
        }
    }

    void setAllSensorsNaN()
    {
        const double nanVal = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 0; i < sensorCount; ++i)
        {
            publisher.updateValue(i, nanVal);
        }
    }

    void refreshAssetOnInsert()
    {
        if (nmClient && invSink)
        {
            nmClient->readAsset(
                psuAddr8,
                [this](std::optional<psu::inventory::PsuAsset> asset) {
                    if (asset && invSink)
                    {
                        invSink->updateAsset(psuName, *asset);
                    }
                });
        }
    }

    // Commit one completed cycle: advance the differentiated dynamic presence
    // tracker (0x80) and consecutive-failure tracker (0xC0/transport stalls),
    // updating Present/Functional/Available across sensor and inventory objects (spec §5.7).
    void applyCycle(const CycleResult& result)
    {
        const auto presenceChange = presenceTracker.record(result.outcome);

        if (result.outcome == ReadOutcome::deviceTimeout)
        {
            // SMLink device timeout (0x80) is exclusively tracked by PresenceTracker.
            // It does not increment FailureTracker.
            if (presenceChange == PresenceChange::removed)
            {
                lastProfile = StateProfile::absent;
                setAllSensorsNaN();
                for (std::size_t i = 0; i < sensorCount; ++i)
                {
                    publisher.updateStatus(i, false, false);
                }
                publishFailureTransition(false, false, false);
                logger.logRemoved(psuName);
            }
            return;
        }

        if (result.outcome == ReadOutcome::nodeBusy ||
            result.outcome == ReadOutcome::transportError)
        {
            // 0xC0 (Node Busy) and IPMB transport errors never modify Present;
            // they only increment FailureTracker toward failuresBeforeNotFunctional.
            tracker.recordFailure();

            if (!presenceTracker.isPresent())
            {
                lastProfile = StateProfile::absent;
                return;
            }

            if (!tracker.functional())
            {
                const bool transitioningToCommFault =
                    (lastProfile != StateProfile::commFault);
                lastProfile = StateProfile::commFault;
                for (std::size_t i = 0; i < sensorCount; ++i)
                {
                    publisher.updateStatus(i, false, false);
                }
                publishFailureTransition(true, false, false);

                if (transitioningToCommFault)
                {
                    std::optional<std::uint8_t> cc = std::nullopt;
                    if (result.outcome == ReadOutcome::nodeBusy)
                    {
                        cc = 0xC0;
                    }
                    logger.logCommsFailure(psuName, cc);
                }
            }
            return;
        }

        // result.outcome == ReadOutcome::success:
        tracker.recordSuccess();

        const auto prevProfile = lastProfile;
        const bool wasAbsent =
            (prevProfile == StateProfile::absent || !lastPresent);
        const bool wasCommFault = (prevProfile == StateProfile::commFault);
        const bool wasHwFault = (prevProfile == StateProfile::hwFault);
        const bool hasFault =
            result.status.has_value() && result.status->hasCriticalFault();

        const auto targetProfile =
            hasFault ? StateProfile::hwFault : StateProfile::operational;
        lastProfile = targetProfile;
        const bool functional = !hasFault;
        const bool available = true;

        for (std::size_t i = 0; i < sensorCount; ++i)
        {
            publisher.updateStatus(i, functional, available);
        }
        publishFailureTransition(true, functional, available);

        if (wasAbsent || presenceChange == PresenceChange::inserted)
        {
            refreshAssetOnInsert();
            logger.logInserted(psuName);
            if (targetProfile == StateProfile::hwFault)
            {
                logger.logHardwareFault(
                    psuName,
                    psu::pmbus::firstCriticalFaultName(*result.status),
                    result.status->raw);
            }
        }
        else if (wasCommFault)
        {
            logger.logRecovery(psuName);
            if (targetProfile == StateProfile::hwFault)
            {
                logger.logHardwareFault(
                    psuName,
                    psu::pmbus::firstCriticalFaultName(*result.status),
                    result.status->raw);
            }
        }
        else if (wasHwFault)
        {
            if (targetProfile == StateProfile::operational)
            {
                logger.logRecovery(psuName);
            }
        }
        else // wasOperational
        {
            if (targetProfile == StateProfile::hwFault)
            {
                logger.logHardwareFault(
                    psuName,
                    psu::pmbus::firstCriticalFaultName(*result.status),
                    result.status->raw);
            }
        }
    }

    // Emit transitions once per crossing and mirror the slot's
    // Present/Functional/Available states onto inventory (spec §5.7).
    void publishFailureTransition(bool present, bool functional, bool available)
    {
        const bool presenceChanged = (present != lastPresent);
        const bool functionalChanged = (functional != lastFunctional);
        const bool availableChanged = (available != lastAvailable);
        if (!presenceChanged && !functionalChanged && !availableChanged)
        {
            return;
        }
        lastPresent = present;
        lastFunctional = functional;
        lastAvailable = available;

        if (onFunctionalChange)
        {
            onFunctionalChange(present, functional, available);
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

        if (functionalChanged || availableChanged)
        {
            if (functional && available)
            {
                std::cerr << psuName << ": operational again\n";
            }
            else if (!available && !functional)
            {
                if (!tracker.functional())
                {
                    std::cerr << psuName << ": communication fault after "
                              << tracker.threshold() << " consecutive failures\n";
                }
                else
                {
                    std::cerr << psuName << ": not available/functional\n";
                }
            }
            else if (available && !functional)
            {
                std::cerr << psuName << ": hardware fault\n";
            }
        }
    }

    std::unique_ptr<IpmbWordReader> ownedReader;
    std::unique_ptr<DbusSensorPublisher> ownedPublisher;
    std::unique_ptr<psu::event::EventLogger> ownedLogger;
    std::string psuName;
    std::uint8_t psuAddr8 = 0;
    std::chrono::milliseconds interval;
    boost::asio::steady_timer timer;
    std::size_t sensorCount = 0;
    int voutExponent = defaultVoutExponent;
    PresenceTracker presenceTracker;
    FailureTracker tracker;
    WordReader& reader;
    SensorPublisher& publisher;
    psu::event::EventLogger& logger;
    PollCycle cycle;
    std::map<std::string, Thresholds> thresholds;
    FunctionalSink onFunctionalChange;
    bool lastPresent = false;
    bool lastFunctional = false;
    bool lastAvailable = false;
    StateProfile lastProfile = StateProfile::absent;
    psu::discovery::NodeManagerClient* nmClient = nullptr;
    psu::discovery::InventorySink* invSink = nullptr;
};

} // namespace psu::sensors
