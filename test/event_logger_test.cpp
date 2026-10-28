#include "event_logger.hpp"
#include "poll_cycle.hpp"
#include "poller.hpp"

#include <gtest/gtest.h>

#include <boost/asio/io_context.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace
{

struct LoggedEvent
{
    std::string message;
    std::string severity;
    std::map<std::string, std::string> metadata;

    bool operator==(const LoggedEvent&) const = default;
};

class FakeEventLogger : public psu::event::EventLogger
{
  public:
    std::vector<LoggedEvent> events;

    void logInserted(const std::string& psuName) override
    {
        events.push_back({
            psu::event::powerSupplyInsertedMessage,
            psu::event::severityInformational,
            {{"PSU", psuName}},
        });
    }

    void logRemoved(const std::string& psuName) override
    {
        events.push_back({
            psu::event::powerSupplyRemovedMessage,
            psu::event::severityWarning,
            {{"PSU", psuName}},
        });
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
        events.push_back({
            psu::event::powerSupplyFailureMessage,
            psu::event::severityWarning,
            {
                {"PSU", psuName},
                {"ERROR", "COMMUNICATION_FAILURE"},
                {"IPMB_CC", ccStr},
            },
        });
    }

    void logHardwareFault(const std::string& psuName,
                          std::string_view faultName,
                          std::uint16_t statusWord) override
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%04X", statusWord);
        events.push_back({
            psu::event::powerSupplyFailureMessage,
            psu::event::severityCritical,
            {
                {"PSU", psuName},
                {"ERROR", std::string(faultName)},
                {"STATUS_WORD", buf},
            },
        });
    }

    void logRecovery(const std::string& psuName) override
    {
        events.push_back({
            psu::event::powerSupplyRecoveryMessage,
            psu::event::severityInformational,
            {{"PSU", psuName}},
        });
    }
};

class ScriptedReader : public psu::sensors::WordReader
{
  public:
    void read(std::uint8_t command,
              std::function<void(psu::sensors::WordResult)> callback) override
    {
        const auto it = responses.find(command);
        callback(it == responses.end() ? std::nullopt : it->second);
    }

    std::map<std::uint8_t, psu::sensors::WordResult> responses;
};

class DummyPublisher : public psu::sensors::SensorPublisher
{
  public:
    void publishSensor(std::size_t, const std::string&,
                       const psu::sensors::SensorDef&, double, bool, bool,
                       const psu::sensors::ThresholdProperties&) override
    {}
    void updateValue(std::size_t, double) override {}
    void updateStatus(std::size_t, bool, bool) override {}
};

const psu::sensors::SensorDef* sensorDef(const char* label)
{
    return psu::sensors::findSensor(label);
}

} // namespace

// AC1: Constant validation and EventLogger interface contract
TEST(EventLoggerContract, ConstantsMatchOpenBmcSpecification)
{
    EXPECT_STREQ(psu::event::powerSupplyInsertedMessage,
                 "OpenBMC.0.1.PowerSupplyInserted");
    EXPECT_STREQ(psu::event::powerSupplyRemovedMessage,
                 "OpenBMC.0.1.PowerSupplyRemoved");
    EXPECT_STREQ(psu::event::powerSupplyFailureMessage,
                 "OpenBMC.0.1.PowerSupplyFailure");
    EXPECT_STREQ(psu::event::powerSupplyRecoveryMessage,
                 "OpenBMC.0.1.PowerSupplyRecovery");

    EXPECT_STREQ(psu::event::severityInformational,
                 "xyz.openbmc_project.Logging.Entry.Level.Informational");
    EXPECT_STREQ(psu::event::severityWarning,
                 "xyz.openbmc_project.Logging.Entry.Level.Warning");
    EXPECT_STREQ(psu::event::severityCritical,
                 "xyz.openbmc_project.Logging.Entry.Level.Critical");
}

// AC2: STATE_ABSENT -> STATE_OPERATIONAL emits PowerSupplyInserted (Informational)
// Boot discovery Tick 1 and runtime hot-plug.
TEST(EventLoggerTransitions, BootTick1DiscoveryEmitsPowerSupplyInserted)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C; // VOUT_MODE
    reader.responses[0x79] = 0x0000; // clean STATUS_WORD
    reader.responses[0x88] = 0x0002; // vin

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    // Slot starts under STATE_ABSENT
    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/false, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::absent);
    EXPECT_TRUE(logger.events.empty());

    // Poller Tick 1 clean read discovers PSU
    poller.start();

    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyInsertedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");

    // Subsequent clean poll does not re-emit
    poller.poll();
    EXPECT_EQ(logger.events.size(), 1U);
}

// AC2: Runtime hot-plug emits PowerSupplyInserted (Informational)
TEST(EventLoggerTransitions, RuntimeHotPlugEmitsPowerSupplyInserted)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // Hot-unplug (2 cycles of 0x80) -> STATE_ABSENT
    reader.responses[0x79] = psu::sensors::ReadOutcome::deviceTimeout;
    poller.poll();
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::absent);
    ASSERT_EQ(logger.events.size(), 1U); // Removed event

    // Hot-insert (0x00 clean) -> STATE_OPERATIONAL
    reader.responses[0x79] = 0x0000;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyInsertedMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[1].metadata.at("PSU"), "PSU1");
}

// AC3: STATE_ABSENT -> STATE_HW_FAULT emits PowerSupplyInserted then PowerSupplyFailure (Critical)
TEST(EventLoggerTransitions, BootTick1DiscoveryWithHardwareFaultEmitsTwoEventsInOrder)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0400; // FAN_FAULT (mask 0x2704)
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/false, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();

    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 2U);

    // 1. PowerSupplyInserted with Severity = Informational
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyInsertedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");

    // 2. PowerSupplyFailure with Severity = Critical and metadata
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityCritical);
    EXPECT_EQ(logger.events[1].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[1].metadata.at("ERROR"), "FAN_FAULT");
    EXPECT_EQ(logger.events[1].metadata.at("STATUS_WORD"), "0x0400");
}

// AC4: Transitioning to STATE_ABSENT after 2 consecutive 0x80 timeouts emits PowerSupplyRemoved (Warning)
TEST(EventLoggerTransitions, TwoConsecutive0x80EmitsPowerSupplyRemoved)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU2", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB2,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // 1st cycle of 0x80: debounce, no event emitted
    reader.responses[0x79] = psu::sensors::ReadOutcome::deviceTimeout;
    poller.poll();
    EXPECT_TRUE(poller.isPresent());
    EXPECT_TRUE(logger.events.empty());

    // 2nd consecutive cycle of 0x80: declared absent -> emits PowerSupplyRemoved
    poller.poll();
    EXPECT_FALSE(poller.isPresent());
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::absent);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRemovedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU2");

    // 3rd cycle: stays absent, does not re-emit
    poller.poll();
    EXPECT_EQ(logger.events.size(), 1U);
}

// AC5: Transitioning to STATE_COMM_FAULT due to consecutive 0xC0 comms failures
// emits PowerSupplyFailure with Warning and [PSU=<name>, ERROR=COMMUNICATION_FAILURE, IPMB_CC=0x%02X]
TEST(EventLoggerTransitions, CommsFailure0xC0EmitsPowerSupplyFailureWarning)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = psu::sensors::ReadOutcome::nodeBusy; // 0xC0

    // Cycle 1 and 2: within tolerance threshold (3), no event
    poller.poll();
    poller.poll();
    EXPECT_TRUE(logger.events.empty());

    // Cycle 3: threshold reached -> STATE_COMM_FAULT
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[0].metadata.at("ERROR"), "COMMUNICATION_FAILURE");
    EXPECT_EQ(logger.events[0].metadata.at("IPMB_CC"), "0xC0");

    // Cycle 4: stays in STATE_COMM_FAULT, does not re-emit
    poller.poll();
    EXPECT_EQ(logger.events.size(), 1U);
}

// AC5: Transitioning to STATE_COMM_FAULT due to transport timeouts
// emits PowerSupplyFailure with IPMB_CC=TRANSPORT_TIMEOUT
TEST(EventLoggerTransitions, TransportTimeoutEmitsPowerSupplyFailureWithTransportTimeout)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    // failure threshold = 2
    psu::sensors::Poller poller(
        io, reader, publisher, "PSU2", std::chrono::milliseconds(1000), 2,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB2,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = psu::sensors::ReadOutcome::transportError;

    poller.poll(); // Cycle 1
    EXPECT_TRUE(logger.events.empty());

    poller.poll(); // Cycle 2 -> threshold reached
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU2");
    EXPECT_EQ(logger.events[0].metadata.at("ERROR"), "COMMUNICATION_FAILURE");
    EXPECT_EQ(logger.events[0].metadata.at("IPMB_CC"), "TRANSPORT_TIMEOUT");
}

// AC6: Transitioning from STATE_OPERATIONAL to STATE_HW_FAULT for each of 5 critical faults (mask 0x2704)
// emits PowerSupplyFailure (Critical) with metadata [PSU=<name>, ERROR=<fault_name>, STATUS_WORD=0x%04X]
TEST(EventLoggerTransitions, OperationalToHardwareFaultAllCriticalBits)
{
    struct FaultCase
    {
        std::uint16_t word;
        std::string faultName;
        std::string wordHex;
    };

    const std::vector<FaultCase> cases = {
        {0x0004, "TEMPERATURE_FAULT", "0x0004"}, // bit 2
        {0x0100, "VOUT_OV_FAULT", "0x0100"},      // bit 8
        {0x0200, "IOUT_OC_FAULT", "0x0200"},      // bit 9
        {0x0400, "FAN_FAULT", "0x0400"},          // bit 10
        {0x2000, "VIN_UV_FAULT", "0x2000"},       // bit 13
    };

    for (const auto& testCase : cases)
    {
        boost::asio::io_context io;
        ScriptedReader reader;
        reader.responses[0x20] = 0x001C;
        reader.responses[0x79] = 0x0000; // start clean operational
        reader.responses[0x88] = 0x0002;

        DummyPublisher publisher;
        FakeEventLogger logger;
        std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

        psu::sensors::Poller poller(
            io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
            defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
            nullptr, nullptr, &logger);

        poller.start();
        logger.events.clear();

        // Critical fault asserted
        reader.responses[0x79] = testCase.word;
        poller.poll();

        EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
        ASSERT_EQ(logger.events.size(), 1U) << "Failed for fault " << testCase.faultName;
        EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailureMessage);
        EXPECT_EQ(logger.events[0].severity, psu::event::severityCritical);
        EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
        EXPECT_EQ(logger.events[0].metadata.at("ERROR"), testCase.faultName);
        EXPECT_EQ(logger.events[0].metadata.at("STATUS_WORD"), testCase.wordHex);

        // Next cycle with same fault does not duplicate log
        poller.poll();
        EXPECT_EQ(logger.events.size(), 1U);
    }
}

// AC7: Transitioning from STATE_COMM_FAULT to STATE_OPERATIONAL on clean poll emits PowerSupplyRecovery (Informational)
TEST(EventLoggerTransitions, CleanPollRecoversFromCommFault)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 1,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // Trigger STATE_COMM_FAULT
    reader.responses[0x79] = psu::sensors::ReadOutcome::nodeBusy;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);

    // Clean poll (0x00 & STATUS_WORD & 0x2704 == 0) -> STATE_OPERATIONAL
    reader.responses[0x79] = 0x0000;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyRecoveryMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[1].metadata.at("PSU"), "PSU1");
}

// AC7: Transitioning from STATE_HW_FAULT to STATE_OPERATIONAL on clean poll emits PowerSupplyRecovery (Informational)
TEST(EventLoggerTransitions, CleanPollRecoversFromHardwareFault)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0400; // FAN_FAULT
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    logger.events.clear();

    // Clean poll clears hardware fault
    reader.responses[0x79] = 0x0000;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRecoveryMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
}

// AC8: Transitioning from STATE_COMM_FAULT to STATE_HW_FAULT on resumed communication with critical fault
// emits exact sequential order:
//   1. OpenBMC.0.1.PowerSupplyRecovery (Informational)
//   2. OpenBMC.0.1.PowerSupplyFailure (Critical) with metadata
TEST(EventLoggerTransitions, ResumedCommsWithHardwareFaultEmitsRecoveryThenFailure)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000;
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 1,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // Enter STATE_COMM_FAULT
    reader.responses[0x79] = psu::sensors::ReadOutcome::nodeBusy;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    logger.events.clear();

    // Resumed comms (0x00) with critical TEMPERATURE_FAULT (0x0004)
    reader.responses[0x79] = 0x0004;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 2U);

    // 1. PowerSupplyRecovery (Informational) closing comms fault
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRecoveryMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");

    // 2. PowerSupplyFailure (Critical) asserting hardware fault
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityCritical);
    EXPECT_EQ(logger.events[1].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[1].metadata.at("ERROR"), "TEMPERATURE_FAULT");
    EXPECT_EQ(logger.events[1].metadata.at("STATUS_WORD"), "0x0004");
}

// Edge case caveat: Severity Downgrade (STATE_HW_FAULT -> STATE_COMM_FAULT)
// Table 3.4.3: Hardware-faulted PSU experiences 3 consecutive comms failures emits PowerSupplyFailure (Warning).
// When comms recovers with unresolved HW fault, re-emits PowerSupplyRecovery and PowerSupplyFailure (Critical).
TEST(EventLoggerTransitions, SeverityDowngradeAndReassertion)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0400; // FAN_FAULT
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 2,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityCritical);
    logger.events.clear();

    // 2 consecutive 0xC0 failures trigger STATE_COMM_FAULT (severity downgrade)
    reader.responses[0x79] = psu::sensors::ReadOutcome::nodeBusy;
    poller.poll();
    EXPECT_TRUE(logger.events.empty());
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("ERROR"), "COMMUNICATION_FAILURE");
    logger.events.clear();

    // Comms recovers with unresolved FAN_FAULT -> re-emits Recovery (Info) and Failure (Critical)
    reader.responses[0x79] = 0x0400;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRecoveryMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyFailureMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityCritical);
    EXPECT_EQ(logger.events[1].metadata.at("ERROR"), "FAN_FAULT");
    EXPECT_EQ(logger.events[1].metadata.at("STATUS_WORD"), "0x0400");
}

// AC9: Routine host S0/S5 power transitions and CML/Warning bits do not emit event logs
TEST(EventLoggerTransitions, RoutinePowerTransitionsAndWarningsDoNotEmitEventLogs)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0000; // S0 running (OFF=0, POWER_GOOD#=0)
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // Transition to S5 standby (OFF=1, POWER_GOOD#=1 -> 0x0840)
    reader.responses[0x79] = 0x0840;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    EXPECT_TRUE(logger.events.empty());

    // CML bit asserted (0x0842)
    reader.responses[0x79] = 0x0842;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    EXPECT_TRUE(logger.events.empty());

    // Warning bits asserted (0x084A: VIN_UV_WARNING bit 3)
    reader.responses[0x79] = 0x084A;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    EXPECT_TRUE(logger.events.empty());

    // Transition back to S0 running (0x0000)
    reader.responses[0x79] = 0x0000;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    EXPECT_TRUE(logger.events.empty());
}
