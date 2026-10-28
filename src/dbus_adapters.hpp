#pragma once

#include "inventory.hpp"
#include "inventory_sink.hpp"
#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "node_manager_client.hpp"
#include "object_mapper_client.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <map>
#include <memory>
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

} // namespace psu::discovery
