#pragma once

#include <sdbusplus/bus.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace sdbusplus::asio
{
class dbus_interface;
class object_server;
} // namespace sdbusplus::asio

namespace psu::inventory
{

// D-Bus surface published for each configured PSU slot (spec §5.6).
inline constexpr auto powerSupplyInterface =
    "xyz.openbmc_project.Inventory.Item.PowerSupply";
inline constexpr auto assetInterface =
    "xyz.openbmc_project.Inventory.Decorator.Asset";
inline constexpr auto itemInterface = "xyz.openbmc_project.Inventory.Item";
inline constexpr auto operationalStatusInterface =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
inline constexpr auto associationInterface =
    "xyz.openbmc_project.Association.Definitions";

// The interfaces bmcweb resolves a Redfish chassis id against before following
// that object's powered_by association (see bmcweb chassisInterfaces).
inline constexpr auto boardInterface =
    "xyz.openbmc_project.Inventory.Item.Board";
inline constexpr auto chassisItemInterface =
    "xyz.openbmc_project.Inventory.Item.Chassis";

inline constexpr auto powerSupplyRoot =
    "/xyz/openbmc_project/inventory/system/power_supply";

// The Asset strings read once from a slot's MFR_ID / MFR_MODEL.
struct PsuAsset
{
    std::string manufacturer;
    std::string model;

    bool operator==(const PsuAsset&) const = default;
};

// One published PSU inventory object. A slot is present when its MFR read
// yielded a usable manufacturer string; the registered-but-empty slot is
// present=false / functional=false (spec §5.6).
struct PsuInventory
{
    std::string name;
    bool present;
    bool functional;
    std::string manufacturer;
    std::string model;

    bool operator==(const PsuInventory&) const = default;
};

// [forward, backward, target] association on the chassis object.
using Association = std::tuple<std::string, std::string, std::string>;

// The interfaces of a published slot that later polls update (spec §5.7). The
// item interface carries `Present`; the status interface `Functional`.
struct PowerSupplyHandles
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> item;
    std::shared_ptr<sdbusplus::asio::dbus_interface> status;
};

// Published slot handles keyed by PSU name.
using PowerSupplyHandleMap = std::map<std::string, PowerSupplyHandles>;

inline std::string powerSupplyPath(std::string_view psuName)
{
    return std::string(powerSupplyRoot) + "/" + std::string(psuName);
}

// Resolve the inventory object bmcweb uses as this system's chassis: among
// candidate paths implementing a Board/Chassis interface, the one whose leaf
// filename equals `chassisId` (bmcweb resolves the Redfish chassis id by that
// filename, see bmcweb chassis_utils). An empty result means no association is
// published, so the powered_by association is never left on an unrelated path.
inline std::string
    selectChassisPath(const std::vector<std::string>& candidatePaths,
                      std::string_view chassisId)
{
    for (const auto& path : candidatePaths)
    {
        std::string_view leaf = path;
        const auto slash = path.rfind('/');
        if (slash != std::string::npos)
        {
            leaf = std::string_view(path).substr(slash + 1);
        }
        if (!leaf.empty() && leaf == chassisId)
        {
            return path;
        }
    }
    return {};
}

// The Board/Chassis object paths under the inventory root, via the
// ObjectMapper. Throws on a D-Bus failure.
std::vector<std::string> findBoardOrChassisPaths(sdbusplus::bus_t& bus);

inline PsuInventory
    makeInventory(std::string name, const std::optional<PsuAsset>& asset)
{
    if (asset)
    {
        return {std::move(name), true, true, asset->manufacturer,
                asset->model};
    }
    return {std::move(name), false, false, {}, {}};
}

// Asset interface properties for an inventory object.
inline std::map<std::string, std::string>
    assetProperties(const PsuInventory& inventory)
{
    return {
        {"Manufacturer", inventory.manufacturer},
        {"Model", inventory.model},
    };
}

// `["powered_by", "powering", <PSU path>]` for the chassis association.
inline Association poweringAssociation(std::string_view psuPath)
{
    return {"powered_by", "powering", std::string(psuPath)};
}

inline std::vector<Association>
    chassisPoweringAssociations(const std::vector<std::string>& psuPaths)
{
    std::vector<Association> associations;
    associations.reserve(psuPaths.size());
    for (const auto& path : psuPaths)
    {
        associations.push_back(poweringAssociation(path));
    }
    return associations;
}

// A decoded Asset string is usable when it is non-empty printable ASCII. An
// empty/absent PSU can otherwise return a length byte that decodes to garbage;
// this is how the registered-but-empty slot is rejected (spec §5.6).
inline bool usableAssetString(std::string_view value)
{
    if (value.empty())
    {
        return false;
    }
    for (const char c : value)
    {
        const auto byte = static_cast<unsigned char>(c);
        if (byte < 0x20 || byte > 0x7E)
        {
            return false;
        }
    }
    return true;
}

// Read a slot's Asset once at startup: MFR_ID (0x99) and MFR_MODEL (0x9A) are
// PMBus BLOCK_READs whose length-prefixed ASCII blocks are decoded with
// psu::pmbus::decodeAscii. Returns no value when MFR_ID is unusable, which is
// how the registered-but-empty slot is detected. The read tolerates the ME's
// non-zero completion code for block reads (see psu::ipmb).
std::optional<PsuAsset> readAsset(sdbusplus::bus_t& bus,
                                  std::uint8_t psuAddr8);

// Publish the inventory objects. Returns the per-slot handles so the poller
// can flip Present/Functional when a slot's failure state changes (spec §5.7).
PowerSupplyHandleMap publish(sdbusplus::asio::object_server& server,
                             const std::vector<PsuInventory>& inventory);

// Attach the powered_by associations for `psuPaths` to `chassisPath` (the
// inventory object bmcweb resolves). A no-op for an empty path, so no
// association is ever left on an unrelated object (spec §5.6).
void publishChassisAssociation(sdbusplus::asio::object_server& server,
                               const std::vector<std::string>& psuPaths,
                               std::string_view chassisPath);

// Flip the Present/Functional pair on a published slot after startup.
void setPresentAndFunctional(const PowerSupplyHandles& handles, bool present,
                             bool functional);

} // namespace psu::inventory
