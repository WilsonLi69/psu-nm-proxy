#include "inventory.hpp"

#include "ipmb_client.hpp"
#include "nm_command.hpp"
#include "pmbus.hpp"
#include "sensor_set.hpp"

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>

#include <cstdint>
#include <exception>
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

std::string readAsciiBlock(sdbusplus::bus_t& bus, std::uint8_t psuAddr8,
                           std::uint8_t command)
{
    const auto payload =
        psu::nm::encodeBlockRead(psuAddr8, command, assetReadLength);
    return psu::pmbus::decodeAscii(
        psu::ipmb::sendRawPmbusToleratingCompletionCode(bus, payload));
}

} // namespace

std::optional<PsuAsset> readAsset(sdbusplus::bus_t& bus, std::uint8_t psuAddr8)
{
    std::string manufacturer;
    try
    {
        manufacturer =
            readAsciiBlock(bus, psuAddr8, psu::sensors::mfrIdCommand);
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }
    if (!usableAssetString(manufacturer))
    {
        return std::nullopt;
    }

    std::string model;
    try
    {
        model = readAsciiBlock(bus, psuAddr8, psu::sensors::mfrModelCommand);
    }
    catch (const std::exception&)
    {
    }
    if (!usableAssetString(model))
    {
        model.clear();
    }

    return PsuAsset{manufacturer, model};
}

std::vector<std::string> findBoardOrChassisPaths(sdbusplus::bus_t& bus)
{
    auto method = bus.new_method_call(objectMapperService, objectMapperPath,
                                      objectMapperInterface, "GetSubTreePaths");
    method.append(inventoryRoot, 0,
                  std::vector<std::string>{boardInterface,
                                           chassisItemInterface});
    auto reply = bus.call(method);
    std::vector<std::string> paths;
    reply.read(paths);
    return paths;
}

PowerSupplyHandleMap publish(sdbusplus::asio::object_server& server,
                             const std::vector<PsuInventory>& inventory)
{
    std::vector<std::string> psuPaths;
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

        auto status = server.add_interface(path, operationalStatusInterface);
        status->register_property("Functional", psu.functional);
        status->initialize();

        handles.emplace(psu.name,
                        PowerSupplyHandles{std::move(item), std::move(status)});
        psuPaths.push_back(path);
    }

    return handles;
}

void publishChassisAssociation(sdbusplus::asio::object_server& server,
                               const std::vector<std::string>& psuPaths,
                               std::string_view chassisPath)
{
    if (chassisPath.empty())
    {
        return;
    }

    auto chassis =
        server.add_interface(std::string(chassisPath), associationInterface);
    chassis->register_property("Associations",
                               chassisPoweringAssociations(psuPaths));
    chassis->initialize();
}

void setPresentAndFunctional(const PowerSupplyHandles& handles, bool present,
                             bool functional)
{
    if (handles.item)
    {
        handles.item->set_property("Present", present);
    }
    if (handles.status)
    {
        handles.status->set_property("Functional", functional);
    }
}

} // namespace psu::inventory
