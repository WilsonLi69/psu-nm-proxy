#pragma once

#include "inventory.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace psu::discovery
{

// Asynchronous transport seam communicating with the Intel Node Manager via IPMB.
class NodeManagerClient
{
  public:
    virtual ~NodeManagerClient() = default;

    using Response = std::optional<std::vector<std::uint8_t>>;
    using ResponseCallback = std::function<void(Response)>;

    // Read DEh Get ME FW Capabilities
    virtual void getCapabilities(ResponseCallback callback) = 0;

    // Read D8h Get PSU Configuration
    virtual void getPsuConfig(ResponseCallback callback) = 0;

    // Write D7h Set PSU Configuration
    virtual void setPsuConfig(const std::vector<std::uint8_t>& payload,
                              ResponseCallback callback) = 0;

    // Read Asset (MFR_ID 0x99 and MFR_MODEL 0x9A block reads)
    virtual void readAsset(
        std::uint8_t psuAddr8,
        std::function<void(std::optional<psu::inventory::PsuAsset>)>
            callback) = 0;
};

} // namespace psu::discovery
