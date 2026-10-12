#pragma once

#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "pmbus.hpp"
#include "sensor_set.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace psu::sensors
{

inline constexpr auto sensorValueInterface = "xyz.openbmc_project.Sensor.Value";
inline constexpr auto operationalStatusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";

// Publishes the §8 sensor set for one PSU and refreshes it every interval.
// Unconditional polling: never gated on host power state (spec §5.4).
class Poller
{
  public:
    Poller(boost::asio::io_context& io, sdbusplus::asio::connection& conn,
           sdbusplus::asio::object_server& server, std::string psuName,
           std::uint8_t psuAddr8, std::chrono::milliseconds interval) :
        conn(conn), server(server), psuName(std::move(psuName)),
        psuAddr8(psuAddr8), interval(interval), timer(io)
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
        for (const auto& def : sensorSet)
        {
            const std::string path = sensorPath(psuName, def);

            auto value = server.add_interface(path, sensorValueInterface);
            value->register_property("Value", 0.0);
            value->register_property("Unit", unitType(def.unit));
            value->initialize();

            auto status = server.add_interface(path, operationalStatusInterface);
            status->register_property("Functional", true);
            status->initialize();

            sensors.push_back(
                Published{&def, std::move(value), std::move(status)});
        }
    }

    void readVoutExponent()
    {
        try
        {
            const auto word = psu::nm::wordFromLittleEndian(
                psu::ipmb::sendRawPmbus(conn, psu::nm::encodeReadWord(
                                                  psuAddr8, voutModeCommand)));
            voutExponent = psu::pmbus::voutModeExponent(
                static_cast<std::uint8_t>(word));
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
        bool functional = true;

        for (auto& sensor : sensors)
        {
            try
            {
                const auto word = psu::nm::wordFromLittleEndian(
                    psu::ipmb::sendRawPmbus(
                        conn, psu::nm::encodeReadWord(psuAddr8,
                                                      sensor.def->command)));
                sensor.value->set_property("Value",
                                           decode(*sensor.def, word));
            }
            catch (const std::exception&)
            {
                functional = false;
            }
        }

        functional = readStatus() && functional;

        for (auto& sensor : sensors)
        {
            sensor.status->set_property("Functional", functional);
        }

        timer.expires_after(interval);
        timer.async_wait([this](const boost::system::error_code&) { poll(); });
    }

    bool readStatus()
    {
        try
        {
            const auto word = psu::nm::wordFromLittleEndian(
                psu::ipmb::sendRawPmbus(conn, psu::nm::encodeReadWord(
                                                  psuAddr8, statusWordCommand)));
            return psu::pmbus::decodeStatusWord(word).powerGood;
        }
        catch (const std::exception&)
        {
            return false;
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
};

} // namespace psu::sensors
