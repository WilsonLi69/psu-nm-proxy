#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace psu::discovery
{

// Asynchronous seam for resolving ObjectMapper subtree paths.
class ObjectMapperClient
{
  public:
    virtual ~ObjectMapperClient() = default;

    using PathsCallback =
        std::function<void(std::optional<std::vector<std::string>>)>;

    // Queries ObjectMapper for Board and Chassis subtree paths.
    virtual void findBoardOrChassisPaths(PathsCallback callback) = 0;
};

} // namespace psu::discovery
