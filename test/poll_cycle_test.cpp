#include "poll_cycle.hpp"

#include "failure_tracker.hpp"
#include "operational_state.hpp"

#include <gtest/gtest.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace
{

// A WordReader that completes each read immediately. Synchronous completion is
// a valid non-blocking implementation, so this exercises the decode logic.
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

// A WordReader that holds every read until the test explicitly completes it.
// This models a slow or absent ME whose reply has not arrived yet.
class DeferredReader : public psu::sensors::WordReader
{
  public:
    void read(std::uint8_t command,
              std::function<void(psu::sensors::WordResult)> callback) override
    {
        commands.push_back(command);
        pending.push_back(std::move(callback));
    }

    void complete(psu::sensors::WordResult result)
    {
        auto callback = std::move(pending.front());
        pending.erase(pending.begin());
        callback(result);
    }

    std::vector<std::uint8_t> commands;
    std::vector<std::function<void(psu::sensors::WordResult)>> pending;
};

const psu::sensors::SensorDef* sensorDef(const char* label)
{
    return psu::sensors::findSensor(label);
}

} // namespace

// The poll cycle must not block the io_context while an Ipmb read is
// outstanding; otherwise the object server is starved and introspection times
// out. Here the read never completes until the test releases it, and the
// io_context must still run unrelated work in the meantime.
TEST(PollCycle, DoesNotBlockTheIoContextWhileAReadIsOutstanding)
{
    boost::asio::io_context io;
    DeferredReader reader;
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};
    psu::sensors::PollCycle cycle(reader, defs);

    bool done = false;
    cycle.run(-4, {},
              [&done](psu::sensors::CycleResult) { done = true; });

    // The cycle issues its first read and returns without waiting for it.
    ASSERT_EQ(reader.pending.size(), 1U);
    EXPECT_EQ(reader.commands.at(0), psu::sensors::sensorSet.at(0).command);

    bool otherWorkRan = false;
    boost::asio::steady_timer timer(io);
    timer.expires_after(std::chrono::milliseconds(1));
    timer.async_wait([&otherWorkRan](const boost::system::error_code&) {
        otherWorkRan = true;
    });
    io.run();

    EXPECT_TRUE(otherWorkRan);
    EXPECT_FALSE(done);

    // A late reply still drives the cycle to completion.
    reader.complete(0x0001); // vin
    ASSERT_EQ(reader.pending.size(), 1U);
    EXPECT_FALSE(done);

    reader.complete(0x0000); // STATUS_WORD all clear (running, power good)
    EXPECT_TRUE(done);
}

TEST(PollCycle, DecodesEverySensorAndStatus)
{
    ScriptedReader reader;
    reader.responses[0x88] = 0x0001; // vin Linear11 -> 1
    reader.responses[0x8B] = 0x0010; // vout Linear16, N=-4 -> 1
    reader.responses[0x79] = 0x0000; // STATUS_WORD all clear (running, power good)

    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin"),
                                                        sensorDef("vout")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    std::vector<std::pair<std::size_t, double>> published;
    cycle.run(-4,
              [&published](std::size_t index, double value) {
                  published.emplace_back(index, value);
              },
              [&result](psu::sensors::CycleResult r) {
                  result = std::move(r);
              });

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->readsSucceeded);
    ASSERT_EQ(result->values.size(), 2U);
    ASSERT_TRUE(result->values[0].has_value());
    ASSERT_TRUE(result->values[1].has_value());
    EXPECT_DOUBLE_EQ(*result->values[0], 1.0);
    EXPECT_DOUBLE_EQ(*result->values[1], 1.0);

    // Values are pushed as their reads complete, not batched to cycle end.
    ASSERT_EQ(published.size(), 2U);
    EXPECT_EQ(published[0], (std::pair<std::size_t, double>{0U, 1.0}));
    EXPECT_EQ(published[1], (std::pair<std::size_t, double>{1U, 1.0}));

    ASSERT_TRUE(result->status.has_value());
    EXPECT_TRUE(result->status->powerGood);
    EXPECT_FALSE(result->status->off);
}

// A failed read records nullopt and marks the cycle failed, but the remaining
// reads still complete: a slow slot does not stall the rest of the cycle.
TEST(PollCycle, FailedReadDoesNotStopTheRestOfTheCycle)
{
    ScriptedReader reader;
    reader.responses[0x88] = 0x0002; // vin OK -> 2
    // iin (0x89) not present -> failure
    reader.responses[0x79] = 0x0000;

    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin"),
                                                        sensorDef("iin")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    cycle.run(-4, {},
              [&result](psu::sensors::CycleResult r) {
                  result = std::move(r);
              });

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->readsSucceeded);
    ASSERT_EQ(result->values.size(), 2U);
    ASSERT_TRUE(result->values[0].has_value());
    EXPECT_DOUBLE_EQ(*result->values[0], 2.0);
    EXPECT_FALSE(result->values[1].has_value());
    EXPECT_TRUE(result->status.has_value());
}

// A failed STATUS_WORD read leaves status unset so the caller can treat the
// cycle as failed, while the sensor values are still delivered.
TEST(PollCycle, MissingStatusLeavesStatusUnset)
{
    ScriptedReader reader;
    reader.responses[0x88] = 0x0001; // vin
    // STATUS_WORD (0x79) not present -> failure

    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    cycle.run(-4, {},
              [&result](psu::sensors::CycleResult r) {
                  result = std::move(r);
              });

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->readsSucceeded);
    EXPECT_FALSE(result->status.has_value());
}

// Verifies that when a poll cycle fails communication, slotHealth evaluates to
// Functional=false while keeping Present=true for an installed PSU.
TEST(PollCycle, CommunicationFailureMaintainsPresentWhileClearingFunctional)
{
    ScriptedReader reader;
    // Missing responses simulate communication failure
    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    cycle.run(-4, {}, [&result](psu::sensors::CycleResult r) {
        result = std::move(r);
    });

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->readsSucceeded);
    EXPECT_FALSE(result->status.has_value());

    // When tracker records consecutive failures and degrades to false:
    psu::sensors::FailureTracker tracker(1);
    tracker.recordFailure();
    EXPECT_FALSE(tracker.functional());

    // Installed PSU maintains Present=true, but Functional is false:
    const auto health = psu::sensors::slotHealth(
        true, tracker.functional(), result->status, result->readsSucceeded);
    EXPECT_TRUE(health.present);
    EXPECT_FALSE(health.functional);
}

// Verifies that a clean poll cycle restores Present=true and Functional=true
// on a slot marked absent.
TEST(PollCycle, CleanPollCycleRestoresPresentAndFunctionalOnAbsentSlot)
{
    ScriptedReader reader;
    reader.responses[0x88] = 0x0001; // vin
    reader.responses[0x79] = 0x0000; // STATUS_WORD all clear

    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    cycle.run(-4, {}, [&result](psu::sensors::CycleResult r) {
        result = std::move(r);
    });

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->readsSucceeded);
    EXPECT_TRUE(result->status.has_value());

    psu::sensors::FailureTracker tracker(3);
    // Successful poll cycle calls recordSuccess
    if (result->readsSucceeded && result->status.has_value())
    {
        tracker.recordSuccess();
    }

    // Evaluating an absent slot with clean cycle immediately restores presence
    const auto health = psu::sensors::slotHealth(
        false, tracker.functional(), result->status, result->readsSucceeded);
    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
    EXPECT_EQ(health.state, psu::sensors::operationalStateEnabled);
}

// Verifies that following transient communication failure, a clean poll cycle
// restores Functional=true immediately via tracker.recordSuccess().
TEST(PollCycle, CleanPollCycleRestoresFunctionalAfterTransientFailure)
{
    psu::sensors::FailureTracker tracker(2);
    tracker.recordFailure();
    tracker.recordFailure();
    ASSERT_FALSE(tracker.functional());

    ScriptedReader reader;
    reader.responses[0x88] = 0x0001; // vin
    reader.responses[0x79] = 0x0000; // STATUS_WORD all clear

    std::vector<const psu::sensors::SensorDef*> defs = {sensorDef("vin")};
    psu::sensors::PollCycle cycle(reader, defs);

    std::optional<psu::sensors::CycleResult> result;
    cycle.run(-4, {}, [&result](psu::sensors::CycleResult r) {
        result = std::move(r);
    });

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->readsSucceeded);
    EXPECT_TRUE(result->status.has_value());

    // Clean cycle resets failure tracker
    tracker.recordSuccess();
    EXPECT_TRUE(tracker.functional());

    const auto health = psu::sensors::slotHealth(
        true, tracker.functional(), result->status, result->readsSucceeded);
    EXPECT_TRUE(health.present);
    EXPECT_TRUE(health.functional);
}
