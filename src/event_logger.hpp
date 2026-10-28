#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace psu::event
{

// Standard Redfish Message IDs for OpenBMC event registry (ADR-0003 & spec §5.7).
inline constexpr auto powerSupplyInsertedMessage =
    "OpenBMC.0.1.PowerSupplyInserted";
inline constexpr auto powerSupplyRemovedMessage =
    "OpenBMC.0.1.PowerSupplyRemoved";
inline constexpr auto powerSupplyFailureMessage =
    "OpenBMC.0.1.PowerSupplyFailure";
inline constexpr auto powerSupplyRecoveryMessage =
    "OpenBMC.0.1.PowerSupplyRecovery";

// Standard phosphor-logging Entry Level strings.
inline constexpr auto severityInformational =
    "xyz.openbmc_project.Logging.Entry.Level.Informational";
inline constexpr auto severityWarning =
    "xyz.openbmc_project.Logging.Entry.Level.Warning";
inline constexpr auto severityCritical =
    "xyz.openbmc_project.Logging.Entry.Level.Critical";

// Abstract interface for emitting Redfish D-Bus event logs via phosphor-logging.
// Allows domain pollers to be tested without requiring a live D-Bus daemon.
class EventLogger
{
  public:
    virtual ~EventLogger() = default;

    // Emits OpenBMC.0.1.PowerSupplyInserted (Level: Informational).
    virtual void logInserted(const std::string& psuName) = 0;

    // Emits OpenBMC.0.1.PowerSupplyRemoved (Level: Warning).
    virtual void logRemoved(const std::string& psuName) = 0;

    // Emits OpenBMC.0.1.PowerSupplyFailure (Level: Warning) for communication loss.
    // AdditionalData contains [PSU=<name>, ERROR=COMMUNICATION_FAILURE, IPMB_CC=0x%02X]
    // (or IPMB_CC=TRANSPORT_TIMEOUT if completionCode is std::nullopt).
    virtual void logCommsFailure(
        const std::string& psuName,
        std::optional<std::uint8_t> completionCode = std::nullopt) = 0;

    // Emits OpenBMC.0.1.PowerSupplyFailure (Level: Critical) for PMBus hardware faults.
    // AdditionalData contains [PSU=<name>, ERROR=<fault_name>, STATUS_WORD=0x%04X].
    virtual void logHardwareFault(const std::string& psuName,
                                  std::string_view faultName,
                                  std::uint16_t statusWord) = 0;

    // Emits OpenBMC.0.1.PowerSupplyRecovery (Level: Informational).
    virtual void logRecovery(const std::string& psuName) = 0;
};

// No-op implementation of EventLogger used when no logger is injected.
class NullEventLogger : public EventLogger
{
  public:
    void logInserted(const std::string&) override {}
    void logRemoved(const std::string&) override {}
    void logCommsFailure(const std::string&,
                         std::optional<std::uint8_t> = std::nullopt) override
    {}
    void logHardwareFault(const std::string&, std::string_view,
                          std::uint16_t) override
    {}
    void logRecovery(const std::string&) override {}
};

} // namespace psu::event
