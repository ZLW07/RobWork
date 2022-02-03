#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation3D.hpp>

using namespace rw::math;

int main (int argc, char** argv)
{
    const Quaternion<> quat = Quaternion<> (sqrt (2) / 2, sqrt (2) / 2, 0, 0);
    std::cout << "Quaternion: " << quat << std::endl;
    const Rotation3D<> rotationFromQuat = quat.toRotation3D ();
    std::cout << "Rotation from Quaternion: " << rotationFromQuat << std::endl;

    const Rotation3D<> rot = Rotation3D<> (-1, 0, 0, 0, 0, 1, 0, 1, 0);
    std::cout << "Rotation: " << rot << std::endl;
    const Quaternion<> quatFromRotation = Quaternion<> (rot);
    std::cout << "Quaternion from Rotation: " << quatFromRotation << std::endl;

    return 0;
}
