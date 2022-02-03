#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/SerialDevice.hpp>

using namespace rw::kinematics;
using rw::math::Transform3D;
using rw::models::SerialDevice;

void fwdKinematics(SerialDevice::Ptr sdevice,
    Frame* frame, MovableFrame* mframe, State& state)
{
    // calculate the transform from one frame to another
    Transform3D<> fTmf = Kinematics::frameTframe(frame, mframe, state);
    // calculate the transform from world to frame
    Transform3D<> wTmf = Kinematics::worldTframe( mframe, state );
    // we can find the world to frame transform by a little jogling
    Transform3D<> wTf = wTmf * inverse(fTmf);
    // test if frame is a dynamic attachable frame
    if( Kinematics::isDAF( mframe ) ){
       // attach mframe to end of serial device
       Kinematics::gripFrame(mframe, sdevice->getEnd(), state);
    }
}
