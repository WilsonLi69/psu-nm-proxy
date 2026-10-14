#include "ipmb_reader.hpp"

#include "ipmb_client.hpp"
#include "nm_command.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
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
        [callback = std::move(callback)](psu::ipmb::Response response) {
            WordResult word;
            if (response)
            {
                try
                {
                    word = psu::nm::wordFromLittleEndian(*response);
                }
                catch (const std::exception&)
                {
                    word = std::nullopt;
                }
            }
            callback(word);
        });
}

} // namespace psu::sensors
