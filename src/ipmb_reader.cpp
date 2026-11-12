#include "ipmb_reader.hpp"

#include "ipmb_client.hpp"
#include "nm_command.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <utility>
#include <vector>

namespace psu::sensors
{

IpmbWordReader::IpmbWordReader(sdbusplus::asio::connection& conn,
                               std::uint8_t psuAddr8) :
    conn(conn), psuAddr8(psuAddr8)
{}

void IpmbWordReader::read(std::uint8_t command,
                          std::function<void(WordResult)> callback)
{
    const auto payload = psu::nm::encodeReadWord(psuAddr8, command);
    psu::ipmb::sendRawPmbusAsync(
        conn, payload,
        [callback = std::move(callback)](psu::ipmb::RawPmbusOutcome outcome) {
            if (!outcome.transportOk)
            {
                // The D-Bus/IPMB call failed, so there is no completion code
                // to report.
                callback(WordResult(ReadOutcome::transportError));
                return;
            }

            // Any returned non-success code travels with the result, so a
            // communication-fault record reports the code the ME sent rather
            // than a generic transport error (spec §5.7.3).
            if (outcome.completionCode != psu::ipmb::ccSuccess)
            {
                callback(
                    WordResult::fromCompletionCode(outcome.completionCode));
                return;
            }

            if (!outcome.payload)
            {
                callback(WordResult(ReadOutcome::transportError));
                return;
            }

            try
            {
                const auto word =
                    psu::nm::wordFromLittleEndian(*outcome.payload);
                callback(WordResult(ReadOutcome::success, word));
            }
            catch (const std::exception&)
            {
                callback(WordResult(ReadOutcome::transportError));
            }
        });
}

} // namespace psu::sensors
