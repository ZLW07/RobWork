#include <rw/math/Constants.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>

using namespace rw::math;

int main(int argc, char** argv) {
    const RPY<> rpy = RPY<>(Pi,Pi/2,0);
    std::cout << "RPY: " << rpy << std::endl;
    const Rotation3D<> rotationFromRPY = rpy.toRotation3D();
    std::cout << "Rotation from RPY: " << rotationFromRPY << std::endl;

    const Rotation3D<> rot = Rotation3D<>(-1,0,0,0,0,1,0,1,0);
    std::cout << "Rotation: " << rot << std::endl;
    const RPY<> rpyFromRotation = RPY<>(rot);
    std::cout << "RPY from Rotation: " << rpyFromRotation << std::endl;

    return 0;
}
