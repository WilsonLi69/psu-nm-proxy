#include "event_logger.hpp"
#include "poll_cycle.hpp"
#include "poller.hpp"

#include <gtest/gtest.h>

#include <boost/asio/io_context.hpp>

#include <algorithm>
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
    std::string resolution;

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
            {},
        });
    }

    void logRemoved(const std::string& psuName) override
    {
        events.push_back({
            psu::event::powerSupplyRemovedMessage,
            psu::event::severityWarning,
            {{"PSU", psuName}},
            "Power supply " + psuName +
                " was removed. Check that it is correctly seated.",
        });
    }

    void logCommunicationLoss(
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
            psu::event::powerSupplyFailedMessage,
            psu::event::severityWarning,
            {
                {"PSU", psuName},
                {"IPMB_CC", ccStr},
            },
            "Power supply " + psuName +
                " is not responding. Check the SMLink and ME path.",
        });
    }

    void logHardwareFault(const std::string& psuName,
                          std::string_view faultName,
                          std::uint16_t statusWord) override
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%04X", statusWord);

        const bool inputPowerLoss = psu::pmbus::isInputPowerLoss(faultName);
        std::string resolution = "Power supply " + psuName + " reported " +
                                 std::string(faultName) + ". ";
        resolution += inputPowerLoss
                          ? "Check the AC input and reseat the unit."
                          : "Check the power supply.";

        events.push_back({
            inputPowerLoss ? psu::event::powerSupplyPowerLostMessage
                           : psu::event::powerSupplyFailedMessage,
            psu::event::severityCritical,
            {
                {"PSU", psuName},
                {"ERROR", std::string(faultName)},
                {"STATUS_WORD", buf},
            },
            resolution,
        });
    }

    void logRecovery(const std::string& psuName,
                     bool wasInputPowerLoss) override
    {
        events.push_back({
            wasInputPowerLoss ? psu::event::powerSupplyPowerRestoredMessage
                              : psu::event::powerSupplyRecoveredMessage,
            psu::event::severityInformational,
            {{"PSU", psuName}},
            {},
        });
    }
};

class ScriptedReader : public psu::sensors::WordReader
{
  public:
    void read(std::uint8_t command,
              std::function<void(psu::sensors::WordResult)> callback) override
    {
        calls.push_back(command);
        const auto it = responses.find(command);
        callback(it == responses.end() ? std::nullopt : it->second);
    }

    std::map<std::uint8_t, psu::sensors::WordResult> responses;
    std::vector<std::uint8_t> calls;
};

class DummyPublisher : public psu::sensors::SensorPublisher
{
  public:
    void publishSensor(std::size_t, const std::string&,
                       const psu::sensors::SensorDef&, double, bool) override
    {}
    void updateValue(std::size_t, double) override {}
    void updateAvailable(std::size_t, bool) override {}
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
    EXPECT_STREQ(psu::event::powerSupplyFailedMessage,
                 "OpenBMC.0.1.PowerSupplyFailed");
    EXPECT_STREQ(psu::event::powerSupplyRecoveredMessage,
                 "OpenBMC.0.1.PowerSupplyRecovered");
    EXPECT_STREQ(psu::event::powerSupplyPowerLostMessage,
                 "OpenBMC.0.1.PowerSupplyPowerLost");
    EXPECT_STREQ(psu::event::powerSupplyPowerRestoredMessage,
                 "OpenBMC.0.1.PowerSupplyPowerRestored");

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
        defs, {}, /*initiallyPresent=*/false, /*psuAddr8=*/0xB0,
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
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

// AC3: STATE_ABSENT -> STATE_HW_FAULT emits PowerSupplyInserted then PowerSupplyFailed (Critical)
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
        defs, {}, /*initiallyPresent=*/false, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();

    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 2U);

    // 1. PowerSupplyInserted with Severity = Informational
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyInsertedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");

    // 2. PowerSupplyFailed with Severity = Critical and metadata
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyFailedMessage);
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB2,
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
// emits PowerSupplyFailed with Warning and [PSU=<name>, IPMB_CC=0x%02X] (no ERROR key)
TEST(EventLoggerTransitions, CommsFailure0xC0EmitsPowerSupplyFailedWarning)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
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
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[0].metadata.count("ERROR"), 0U);
    EXPECT_EQ(logger.events[0].metadata.at("IPMB_CC"), "0xC0");

    // Cycle 4: stays in STATE_COMM_FAULT, does not re-emit
    poller.poll();
    EXPECT_EQ(logger.events.size(), 1U);
}

// AC5: An unrecognized non-success completion code is reported as that code,
// not as a transport error: PowerSupplyFailed (Warning) with IPMB_CC=0x02.
TEST(EventLoggerTransitions, UnknownCompletionCodeEmitsPowerSupplyFailedWithThatCode)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // 0x02 is not a code the daemon gives special meaning to.
    reader.responses[0x79] = psu::sensors::WordResult::fromCompletionCode(0x02);

    poller.poll();
    poller.poll();
    EXPECT_TRUE(logger.events.empty());

    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[0].metadata.at("IPMB_CC"), "0x02");
}

// AC5: Transitioning to STATE_COMM_FAULT due to transport timeouts
// emits PowerSupplyFailed with IPMB_CC=TRANSPORT_TIMEOUT
TEST(EventLoggerTransitions, TransportTimeoutEmitsPowerSupplyFailedWithTransportTimeout)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB2,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = psu::sensors::ReadOutcome::transportError;

    poller.poll(); // Cycle 1
    EXPECT_TRUE(logger.events.empty());

    poller.poll(); // Cycle 2 -> threshold reached
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU2");
    EXPECT_EQ(logger.events[0].metadata.count("ERROR"), 0U);
    EXPECT_EQ(logger.events[0].metadata.at("IPMB_CC"), "TRANSPORT_TIMEOUT");
}

// AC6: Transitioning from STATE_OPERATIONAL to STATE_HW_FAULT for each of 5 critical faults (mask 0x2704)
// emits PowerSupplyFailed (Critical) with metadata [PSU=<name>, ERROR=<fault_name>, STATUS_WORD=0x%04X]
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
            defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
            nullptr, nullptr, &logger);

        poller.start();
        logger.events.clear();

        // Critical fault asserted
        reader.responses[0x79] = testCase.word;
        poller.poll();

        EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
        ASSERT_EQ(logger.events.size(), 1U) << "Failed for fault " << testCase.faultName;
        EXPECT_EQ(logger.events[0].message,
                  testCase.faultName == "VIN_UV_FAULT"
                      ? psu::event::powerSupplyPowerLostMessage
                      : psu::event::powerSupplyFailedMessage);
        EXPECT_EQ(logger.events[0].severity, psu::event::severityCritical);
        EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
        EXPECT_EQ(logger.events[0].metadata.at("ERROR"), testCase.faultName);
        EXPECT_EQ(logger.events[0].metadata.at("STATUS_WORD"), testCase.wordHex);
        EXPECT_NE(logger.events[0].resolution.find("PSU1"), std::string::npos);

        // Next cycle with same fault does not duplicate log
        poller.poll();
        EXPECT_EQ(logger.events.size(), 1U);
    }
}

// AC7: Transitioning from STATE_COMM_FAULT to STATE_OPERATIONAL on clean poll emits PowerSupplyRecovered (Informational)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
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
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyRecoveredMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[1].metadata.at("PSU"), "PSU1");
}

// AC7: Transitioning from STATE_HW_FAULT to STATE_OPERATIONAL on clean poll emits PowerSupplyRecovered (Informational)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    logger.events.clear();

    // Clean poll clears hardware fault
    reader.responses[0x79] = 0x0000;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::operational);
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRecoveredMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
}

// Leaving STATE_COMM_FAULT into STATE_HW_FAULT while no episode is open starts
// one: a single fault record, and no communication-recovery record.
TEST(EventLoggerTransitions, ResumedCommsIntoNewHardwareFaultEmitsOnlyTheFaultRecord)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    // Enter STATE_COMM_FAULT while the supply is fault-free: no episode is open.
    reader.responses[0x79] = psu::sensors::ReadOutcome::nodeBusy;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::commFault);
    ASSERT_EQ(logger.events.size(), 1U);
    logger.events.clear();

    // Resumed comms (0x00) with critical TEMPERATURE_FAULT (0x0004): the
    // episode opens, and the communication loss is not separately reported.
    reader.responses[0x79] = 0x0004;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 1U);

    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityCritical);
    EXPECT_EQ(logger.events[0].metadata.at("PSU"), "PSU1");
    EXPECT_EQ(logger.events[0].metadata.at("ERROR"), "TEMPERATURE_FAULT");
    EXPECT_EQ(logger.events[0].metadata.at("STATUS_WORD"), "0x0004");
}

// A fault episode stays open across an intervening communication fault: when
// communication returns with the same fault still asserted, nothing is
// announced (spec §5.7.1 rule 2).
TEST(EventLoggerTransitions, CommsInterruptionWhileFaultedEmitsNothingOnResume)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
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
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
    EXPECT_EQ(logger.events[0].metadata.count("ERROR"), 0U);
    logger.events.clear();

    // Comms recovers with the same FAN_FAULT still asserted: the episode never
    // closed, so neither a recovery nor a second fault record is emitted.
    reader.responses[0x79] = 0x0400;
    poller.poll();
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    EXPECT_TRUE(logger.events.empty());
}

// One fault record per episode, however long the fault persists.
TEST(EventLoggerTransitions, FaultSpanningManyCyclesEmitsExactlyOneRecord)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = 0x0400; // FAN_FAULT
    for (int cycle = 0; cycle < 6; ++cycle)
    {
        poller.poll();
    }

    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].metadata.at("ERROR"), "FAN_FAULT");
}

// The asserted bit set changing inside an open episode announces nothing.
TEST(EventLoggerTransitions, ChangingAssertedBitsWhileOpenEmitsNothing)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C;
    reader.responses[0x79] = 0x0400; // FAN_FAULT opens the episode
    reader.responses[0x88] = 0x0002;

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    ASSERT_EQ(logger.events.size(), 1U);
    logger.events.clear();

    reader.responses[0x79] = 0x0600; // FAN_FAULT + IOUT_OC_FAULT
    poller.poll();
    reader.responses[0x79] = 0x0200; // FAN_FAULT cleared, IOUT_OC_FAULT stays
    poller.poll();
    reader.responses[0x79] = 0x2000; // VIN_UV_FAULT appears instead
    poller.poll();

    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    EXPECT_TRUE(logger.events.empty());
}

// Removal terminates an open episode silently: the removal record is its
// terminator, not a recovery.
TEST(EventLoggerTransitions, RemovalWhileFaultedEmitsRemovedWithoutRecovery)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    ASSERT_EQ(logger.events.size(), 1U);
    logger.events.clear();

    // 2 consecutive 0x80 cycles remove the slot while the episode is open.
    reader.responses[0x79] = psu::sensors::ReadOutcome::deviceTimeout;
    poller.poll();
    poller.poll();
    ASSERT_FALSE(poller.isPresent());

    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyRemovedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityWarning);
}

// A re-inserted unit that is still faulty announces insertion and opens a new
// episode, because removal re-armed the latch.
TEST(EventLoggerTransitions, ReinsertingAStillFaultyUnitEmitsInsertedAndANewFault)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    reader.responses[0x79] = psu::sensors::ReadOutcome::deviceTimeout;
    poller.poll();
    poller.poll();
    ASSERT_FALSE(poller.isPresent());
    logger.events.clear();

    // The same unit returns, still reporting FAN_FAULT.
    reader.responses[0x79] = 0x0400;
    poller.poll();

    EXPECT_TRUE(poller.isPresent());
    EXPECT_EQ(poller.getProfile(), psu::sensors::StateProfile::hwFault);
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyInsertedMessage);
    EXPECT_EQ(logger.events[0].severity, psu::event::severityInformational);
    EXPECT_EQ(logger.events[1].message, psu::event::powerSupplyFailedMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityCritical);
    EXPECT_EQ(logger.events[1].metadata.at("ERROR"), "FAN_FAULT");
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
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

// The daemon observes faults; it must never mutate the supply's fault state.
// A full poll cycle (clean, faulted, standby) issues no CLEAR_FAULTS (0x03).
TEST(EventLoggerTransitions, PollCyclesNeverIssueClearFaults)
{
    boost::asio::io_context io;
    ScriptedReader reader;
    reader.responses[0x20] = 0x001C; // VOUT_MODE
    reader.responses[0x79] = 0x0000; // clean STATUS_WORD
    reader.responses[0x88] = 0x0002; // vin

    DummyPublisher publisher;
    FakeEventLogger logger;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};

    psu::sensors::Poller poller(
        io, reader, publisher, "PSU1", std::chrono::milliseconds(1000), 3,
        defs, {}, /*initiallyPresent=*/false, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    poller.poll();

    reader.responses[0x79] = 0x2000; // VIN_UV_FAULT
    poller.poll();

    reader.responses[0x79] = 0x0840; // S5 standby
    poller.poll();

    ASSERT_FALSE(reader.calls.empty());
    EXPECT_EQ(std::count(reader.calls.begin(), reader.calls.end(), 0x03), 0);
}

TEST(EventLoggerTransitions, HardwareFaultRecordNamesTheSupplyInResolution)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = 0x0400; // FAN_FAULT
    poller.poll();

    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message, psu::event::powerSupplyFailedMessage);
    EXPECT_NE(logger.events[0].resolution.find("PSU1"), std::string::npos);
    EXPECT_NE(logger.events[0].resolution.find("FAN_FAULT"),
              std::string::npos);
}

TEST(EventLoggerTransitions, InputPowerLossRecoveryEmitsPowerRestored)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = 0x2000; // VIN_UV_FAULT: input power loss
    poller.poll();
    ASSERT_EQ(logger.events.size(), 1U);
    EXPECT_EQ(logger.events[0].message,
              psu::event::powerSupplyPowerLostMessage);

    reader.responses[0x79] = 0x0000; // fault observed cleared
    poller.poll();
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[1].message,
              psu::event::powerSupplyPowerRestoredMessage);
    EXPECT_EQ(logger.events[1].severity, psu::event::severityInformational);

    // The episode closed; further fault-free cycles are silent.
    poller.poll();
    EXPECT_EQ(logger.events.size(), 2U);
}

TEST(EventLoggerTransitions, NonInputFaultRecoveryEmitsPowerSupplyRecovered)
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
        defs, {}, /*initiallyPresent=*/true, /*psuAddr8=*/0xB0,
        nullptr, nullptr, &logger);

    poller.start();
    logger.events.clear();

    reader.responses[0x79] = 0x0200; // IOUT_OC_FAULT
    poller.poll();
    ASSERT_EQ(logger.events.size(), 1U);

    reader.responses[0x79] = 0x0000;
    poller.poll();
    ASSERT_EQ(logger.events.size(), 2U);
    EXPECT_EQ(logger.events[1].message,
              psu::event::powerSupplyRecoveredMessage);
}
