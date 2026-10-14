#pragma once

#include "pmbus_reader.hpp"

#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <functional>

namespace psu::sensors
{

// WordReader backed by the IPMB D-Bus channel: every read is an asynchronous
// D-Bus method call on the shared io_context, so the io_context thread is never
// blocked waiting for the ME. See psu::ipmb::sendRawPmbusAsync.
class IpmbWordReader : public WordReader
{
  public:
    IpmbWordReader(sdbusplus::asio::connection& conn, std::uint8_t psuAddr8);

    void read(std::uint8_t command,
              std::function<void(WordResult)> callback) override;

  private:
    sdbusplus::asio::connection& conn;
    std::uint8_t psuAddr8;
};

} // namespace psu::sensors
