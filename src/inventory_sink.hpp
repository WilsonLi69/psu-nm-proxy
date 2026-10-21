#pragma once

#include "inventory.hpp"

#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace psu::discovery
{

// Sink for publishing inventory objects and chassis associations.
class InventorySink
{
  public:
    virtual ~InventorySink() = default;

    using HealthSink = std::function<void(bool present, bool functional)>;

    // Publishes inventory objects and returns a map of slot name to health updater.
    virtual std::map<std::string, HealthSink>
        publishInventory(const std::vector<psu::inventory::PsuInventory>& inventory) = 0;

    // Publishes the powered_by and all_sensors associations onto chassisPath.
    virtual void publishChassisAssociation(
        const std::vector<std::string>& psuPaths, std::string_view chassisPath,
        const std::vector<std::string>& sensorPaths = {}) = 0;
};

} // namespace psu::discovery
