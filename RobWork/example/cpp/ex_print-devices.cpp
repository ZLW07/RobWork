#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::models;

void printDeviceNames(const WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:" << std::endl;
    for(const Device::CPtr device : workcell.getDevices()) {
        std::cout << "- " << device->getName() << std::endl;
    }
}
