#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/SerialDevice.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using rw::models::SerialDevice;

void fwdKinematicsDevice(SerialDevice::Ptr sdevice, MovableFrame* mframe, State& state)
{
    // get device base frame
    Frame *base = sdevice->getBase();
    // get device end effector
    Frame *end = sdevice->getEnd();
    // calculate base to end transform
    Transform3D<> bTe = sdevice->baseTend(state);
    // or just base to any frame
    Transform3D<> bTmf = sdevice->baseTframe(mframe, state);
    // get device name
    std::string sdevicename = sdevice->getName();
    // the degrees of freedom of this device
    int dof = sdevice->getDOF();
    // set the configuration of the device to zero
    sdevice->setQ( Q::zero(dof) , state );
}
