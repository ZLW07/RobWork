#include <rw/core/Log.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

using rw::core::Log;
using namespace rw::kinematics;
using namespace rw::models;

void findFromWorkCell(WorkCell::Ptr wc)
{
    // get the default state
    State state = wc->getDefaultState();
    Frame* worldFrame = wc->getWorldFrame();
    // find a frame by name, remember NULL is a valid return
    Frame* frame = wc->findFrame("FixedFrameName");
    // find a frame by name, but with a specific frame type
    FixedFrame* fframe = wc->findFrame<FixedFrame>("FixedFrameName");
    MovableFrame* mframe = wc->findFrame<MovableFrame>("MovableFrameName");
    // find a device by name
    Device::Ptr device = wc->findDevice("SerialDeviceName");
    SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>("SerialDeviceName");
}
