#pragma once

#include "event_logger.hpp"
#include "inventory.hpp"
#include "inventory_sink.hpp"
#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "node_manager_client.hpp"
#include "object_mapper_client.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace psu::discovery
{

class DbusNodeManagerClient : public NodeManagerClient
{
  public:
    explicit DbusNodeManagerClient(sdbusplus::asio::connection& conn) :
        conn(conn)
    {}

    void getCapabilities(ResponseCallback callback) override
    {
        psu::ipmb::sendCommandAsync(conn, psu::nm::getFwCapabilitiesCmd,
                                    psu::nm::encodeGetFwCapabilities(),
                                    std::move(callback));
    }

    void getPsuConfig(ResponseCallback callback) override
    {
        psu::ipmb::sendCommandAsync(conn, psu::nm::getPsuConfigCmd,
                                    psu::nm::encodeGetPsuConfig(),
                                    std::move(callback));
    }

    void setPsuConfig(const std::vector<std::uint8_t>& payload,
                      ResponseCallback callback) override
    {
        psu::ipmb::sendCommandAsync(conn, psu::nm::setPsuConfigCmd, payload,
                                    std::move(callback));
    }

    void readAsset(
        std::uint8_t psuAddr8,
        std::function<void(std::optional<psu::inventory::PsuAsset>)> callback)
        override
    {
        psu::inventory::readAssetAsync(conn, psuAddr8, std::move(callback));
    }

  private:
    sdbusplus::asio::connection& conn;
};

class DbusObjectMapperClient : public ObjectMapperClient
{
  public:
    explicit DbusObjectMapperClient(sdbusplus::asio::connection& conn) :
        conn(conn)
    {}

    void findBoardOrChassisPaths(PathsCallback callback) override
    {
        psu::inventory::findBoardOrChassisPathsAsync(conn, std::move(callback));
    }

  private:
    sdbusplus::asio::connection& conn;
};

class DbusInventorySink : public InventorySink
{
  public:
    explicit DbusInventorySink(sdbusplus::asio::object_server& server) :
        server(server)
    {}

    std::map<std::string, HealthSink> publishInventory(
        const std::vector<psu::inventory::PsuInventory>& inventory) override
    {
        handles = psu::inventory::publish(server, inventory);
        std::map<std::string, HealthSink> result;
        for (const auto& [name, handle] : handles)
        {
            result[name] = [handle](bool present, bool functional,
                                    std::optional<bool> available) {
                psu::inventory::setPresentAndFunctional(handle, present,
                                                        functional, available);
            };
        }
        return result;
    }

    void publishChassisAssociation(
        const std::vector<std::string>& psuPaths, std::string_view chassisPath,
        const std::vector<std::string>& sensorPaths = {}) override
    {
        psu::inventory::publishChassisAssociation(server, psuPaths,
                                                  chassisPath, sensorPaths);
    }

    void updateAsset(const std::string& name,
                     const psu::inventory::PsuAsset& asset) override
    {
        auto it = handles.find(name);
        if (it != handles.end())
        {
            psu::inventory::setAsset(it->second, asset);
        }
    }

  private:
    sdbusplus::asio::object_server& server;
    psu::inventory::PowerSupplyHandleMap handles;
};

// D-Bus event logger adapter invoking xyz.openbmc_project.Logging.Create on
// /xyz/openbmc_project/logging (ADR-0003 & spec §5.7).
class DbusEventLogger : public psu::event::EventLogger
{
  public:
    explicit DbusEventLogger(sdbusplus::asio::connection& conn) : conn(conn) {}

    void logInserted(const std::string& psuName) override
    {
        emitLog(psu::event::powerSupplyInsertedMessage,
                psu::event::severityInformational,
                {{"PSU", psuName}});
    }

    void logRemoved(const std::string& psuName) override
    {
        emitLog(psu::event::powerSupplyRemovedMessage,
                psu::event::severityWarning,
                {{"PSU", psuName}});
    }

    void logCommsFailure(
        const std::string& psuName,
        std::optional<std::uint8_t> completionCode = std::nullopt) override
    {
        std::string ccStr = "TRANSPORT_TIMEOUT";
        if (completionCode)
        {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "0x%02X", *completionCode);
            ccStr = buf;
        }
        emitLog(psu::event::powerSupplyFailureMessage,
                psu::event::severityWarning,
                {{"PSU", psuName},
                 {"ERROR", "COMMUNICATION_FAILURE"},
                 {"IPMB_CC", ccStr}});
    }

    void logHardwareFault(const std::string& psuName,
                          std::string_view faultName,
                          std::uint16_t statusWord) override
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%04X", statusWord);
        emitLog(psu::event::powerSupplyFailureMessage,
                psu::event::severityCritical,
                {{"PSU", psuName},
                 {"ERROR", std::string(faultName)},
                 {"STATUS_WORD", buf}});
    }

    void logRecovery(const std::string& psuName) override
    {
        emitLog(psu::event::powerSupplyRecoveryMessage,
                psu::event::severityInformational,
                {{"PSU", psuName}});
    }

  private:
    void emitLog(const std::string& message, const std::string& severity,
                 const std::map<std::string, std::string>& additionalData)
    {
        std::string psu;
        auto it = additionalData.find("PSU");
        if (it != additionalData.end())
        {
            psu = it->second;
        }

        conn.async_method_call(
            [message, psu](const boost::system::error_code& ec) {
                if (ec)
                {
                    std::cerr << "psu-nm-proxy: failed to log " << message
                              << " for " << psu << ": " << ec.message() << "\n";
                }
            },
            "xyz.openbmc_project.Logging",
            "/xyz/openbmc_project/logging",
            "xyz.openbmc_project.Logging.Create",
            "Create",
            message,
            severity,
            additionalData);
    }

    sdbusplus::asio::connection& conn;
};

} // namespace psu::discovery

namespace psu::event
{
using DbusEventLogger = psu::discovery::DbusEventLogger;
}
