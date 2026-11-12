#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace psu::event
{

// Registry-defined OpenBMC message identifiers (spec §5.7.2 / Doc 2 §4.4).
// Only these identifiers exist in the platform registry; PowerSupplyFailure and
// PowerSupplyRecovery are not defined and must not be used.
inline constexpr auto powerSupplyInsertedMessage =
    "OpenBMC.0.1.PowerSupplyInserted";
inline constexpr auto powerSupplyRemovedMessage =
    "OpenBMC.0.1.PowerSupplyRemoved";
inline constexpr auto powerSupplyFailedMessage = "OpenBMC.0.1.PowerSupplyFailed";
inline constexpr auto powerSupplyRecoveredMessage =
    "OpenBMC.0.1.PowerSupplyRecovered";
inline constexpr auto powerSupplyPowerLostMessage =
    "OpenBMC.0.1.PowerSupplyPowerLost";
inline constexpr auto powerSupplyPowerRestoredMessage =
    "OpenBMC.0.1.PowerSupplyPowerRestored";

// Standard phosphor-logging Entry Level strings.
inline constexpr auto severityInformational =
    "xyz.openbmc_project.Logging.Entry.Level.Informational";
inline constexpr auto severityWarning =
    "xyz.openbmc_project.Logging.Entry.Level.Warning";
inline constexpr auto severityCritical =
    "xyz.openbmc_project.Logging.Entry.Level.Critical";

// Abstract interface for emitting event records via phosphor-logging.
// Allows domain pollers to be tested without requiring a live D-Bus daemon.
//
// The message text is the identifier alone. Diagnostics travel in
// AdditionalData and a remediation sentence naming the supply travels in the
// entry's Resolution property (spec §5.7.3).
class EventLogger
{
  public:
    virtual ~EventLogger() = default;

    // OpenBMC.0.1.PowerSupplyInserted (Informational).
    virtual void logInserted(const std::string& psuName) = 0;

    // OpenBMC.0.1.PowerSupplyRemoved (Warning).
    virtual void logRemoved(const std::string& psuName) = 0;

    // OpenBMC.0.1.PowerSupplyFailed (Warning) for communication loss.
    // AdditionalData contains [PSU=<name>, IPMB_CC=0x%02X] or
    // IPMB_CC=TRANSPORT_TIMEOUT when completionCode is std::nullopt.
    virtual void logCommunicationLoss(
        const std::string& psuName,
        std::optional<std::uint8_t> completionCode = std::nullopt) = 0;

    // OpenBMC.0.1.PowerSupplyPowerLost (Critical) when the fault is
    // input-power loss, otherwise OpenBMC.0.1.PowerSupplyFailed (Critical).
    // AdditionalData contains [PSU=<name>, ERROR=<fault_name>, STATUS_WORD=0x%04X].
    virtual void logHardwareFault(const std::string& psuName,
                                  std::string_view faultName,
                                  std::uint16_t statusWord) = 0;

    // OpenBMC.0.1.PowerSupplyPowerRestored (Informational) for an input-power
    // loss episode, otherwise OpenBMC.0.1.PowerSupplyRecovered (Informational).
    virtual void logRecovery(const std::string& psuName,
                             bool wasInputPowerLoss) = 0;
};

// No-op implementation of EventLogger used when no logger is injected.
class NullEventLogger : public EventLogger
{
  public:
    void logInserted(const std::string&) override {}
    void logRemoved(const std::string&) override {}
    void logCommunicationLoss(const std::string&,
                              std::optional<std::uint8_t> = std::nullopt) override
    {}
    void logHardwareFault(const std::string&, std::string_view,
                          std::uint16_t) override
    {}
    void logRecovery(const std::string&, bool) override {}
};

} // namespace psu::event
