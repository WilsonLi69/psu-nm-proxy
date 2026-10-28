#include "inventory.hpp"

#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "pmbus.hpp"
#include "sensor_set.hpp"

#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace psu::inventory
{

namespace
{

// MFR_ID/MFR_MODEL BLOCK_READs are asked for 16 bytes, enough for both the
// ACBEL "ACBEL" and "R1BA2751A-P00A" strings (observed on the board).
constexpr std::uint8_t assetReadLength = 0x10;

constexpr auto objectMapperService = "xyz.openbmc_project.ObjectMapper";
constexpr auto objectMapperPath = "/xyz/openbmc_project/object_mapper";
constexpr auto objectMapperInterface = "xyz.openbmc_project.ObjectMapper";
constexpr auto inventoryRoot = "/xyz/openbmc_project/inventory";

// One asynchronous MFR_* BLOCK_READ, decoded to its ASCII string.
void readAsciiBlockAsync(sdbusplus::asio::connection& conn,
                         std::uint8_t psuAddr8, std::uint8_t command,
                         std::function<void(std::optional<std::string>)> cb)
{
    const auto payload =
        psu::nm::encodeBlockRead(psuAddr8, command, assetReadLength);
    psu::ipmb::sendRawPmbusToleratingCompletionCodeAsync(
        conn, payload, [cb = std::move(cb)](psu::ipmb::Response response) {
            if (!response)
            {
                cb(std::nullopt);
                return;
            }
            try
            {
                cb(psu::pmbus::decodeAscii(*response));
            }
            catch (const std::exception&)
            {
                cb(std::nullopt);
            }
        });
}

} // namespace

void readAssetAsync(sdbusplus::asio::connection& conn, std::uint8_t psuAddr8,
                    AssetCallback callback)
{
    readAsciiBlockAsync(
        conn, psuAddr8, psu::sensors::mfrIdCommand,
        [conn = &conn, psuAddr8,
         callback = std::move(callback)](std::optional<std::string> mfrId) mutable {
            if (!mfrId || !usableAssetString(*mfrId))
            {
                callback(std::nullopt);
                return;
            }

            std::string manufacturer = std::move(*mfrId);
            readAsciiBlockAsync(
                *conn, psuAddr8, psu::sensors::mfrModelCommand,
                [manufacturer = std::move(manufacturer),
                 callback = std::move(callback)](
                    std::optional<std::string> model) mutable {
                    if (!model || !usableAssetString(*model))
                    {
                        model = std::string{};
                    }
                    callback(
                        PsuAsset{std::move(manufacturer), std::move(*model)});
                });
        });
}

void findBoardOrChassisPathsAsync(sdbusplus::asio::connection& conn,
                                  PathsCallback callback)
{
    conn.async_method_call(
        [callback = std::move(callback)](const boost::system::error_code& ec,
                                         std::vector<std::string> paths) {
            if (ec)
            {
                callback(std::nullopt);
                return;
            }
            callback(std::move(paths));
        },
        objectMapperService, objectMapperPath, objectMapperInterface,
        "GetSubTreePaths", std::string(inventoryRoot), 0,
        std::vector<std::string>{boardInterface, chassisItemInterface});
}

PowerSupplyHandleMap publish(sdbusplus::asio::object_server& server,
                             const std::vector<PsuInventory>& inventory)
{
    PowerSupplyHandleMap handles;

    for (const auto& psu : inventory)
    {
        const std::string path = powerSupplyPath(psu.name);

        auto powerSupply = server.add_interface(path, powerSupplyInterface);
        powerSupply->initialize();

        auto asset = server.add_interface(path, assetInterface);
        for (const auto& [key, value] : assetProperties(psu))
        {
            asset->register_property(key, value);
        }
        asset->initialize();

        auto item = server.add_interface(path, itemInterface);
        item->register_property("Present", psu.present);
        item->initialize();

        auto avail = server.add_interface(path, availabilityInterface);
        avail->register_property("Available", psu.available);
        avail->initialize();

        auto status = server.add_interface(path, operationalStatusInterface);
        status->register_property("Functional", psu.functional);
        status->initialize();

        handles.emplace(psu.name,
                        PowerSupplyHandles{std::move(item), std::move(status),
                                           std::move(avail), std::move(asset)});
    }

    return handles;
}

void publishChassisAssociation(sdbusplus::asio::object_server& server,
                               const std::vector<std::string>& psuPaths,
                               std::string_view chassisPath,
                               const std::vector<std::string>& sensorPaths)
{
    if (chassisPath.empty())
    {
        return;
    }

    auto chassis =
        server.add_interface(std::string(chassisPath), associationInterface);
    chassis->register_property("Associations",
                               chassisAssociations(psuPaths, sensorPaths));
    chassis->initialize();
}

void setPresentAndFunctional(const PowerSupplyHandles& handles, bool present,
                             bool functional,
                             std::optional<bool> available)
{
    if (handles.item)
    {
        handles.item->set_property("Present", present);
    }
    if (handles.status)
    {
        handles.status->set_property("Functional", functional);
    }
    if (handles.availability)
    {
        const bool isAvail = available.value_or(present && functional);
        handles.availability->set_property("Available", isAvail);
    }
}

void setAsset(const PowerSupplyHandles& handles, const PsuAsset& asset)
{
    if (handles.asset)
    {
        handles.asset->set_property("Manufacturer", asset.manufacturer);
        handles.asset->set_property("Model", asset.model);
    }
}

} // namespace psu::inventory
