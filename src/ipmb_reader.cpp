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
                callback(WordResult(ReadOutcome::transportError));
                return;
            }

            if (outcome.completionCode == psu::ipmb::ccDeviceTimeout)
            {
                callback(WordResult(ReadOutcome::deviceTimeout));
                return;
            }

            if (outcome.completionCode == psu::ipmb::ccNodeBusy)
            {
                callback(WordResult(ReadOutcome::nodeBusy));
                return;
            }

            if (outcome.completionCode != psu::ipmb::ccSuccess ||
                !outcome.payload)
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
