#include <sdbusplus/bus.hpp>

#include <iostream>

int main()
{
    auto bus = sdbusplus::bus::new_default();
    bus.request_name("xyz.openbmc_project.PsuNmProxy");

    std::cout << "psu-nm-proxy: started (no PSU behaviour yet)\n";

    while (true)
    {
        bus.process_discard();
        bus.wait();
    }

    return 0;
}
