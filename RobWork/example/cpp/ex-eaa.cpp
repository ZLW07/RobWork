#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3D.hpp>

using namespace rw::math;

int main(int argc, char** argv) {
    const EAA<> eaa = EAA<>(sqrt(2)/2*Pi,sqrt(2)/2*Pi,0);
    std::cout << "EAA: " << eaa << std::endl;
    std::cout << " angle: " << eaa.angle() << std::endl;
    std::cout << " axis: " << eaa.axis() << std::endl;
    const Rotation3D<> rotationFromEAA = eaa.toRotation3D();
    std::cout << "Rotation from EAA: " << rotationFromEAA << std::endl;

    const Rotation3D<> rot = Rotation3D<>(-1,0,0,0,0,1,0,1,0);
    std::cout << "Rotation: " << rot << std::endl;
    const EAA<> eaaFromRotation = EAA<>(rot);
    std::cout << "EAA from Rotation: " << eaaFromRotation << std::endl;
    std::cout << " angle: " << eaaFromRotation.angle() << std::endl;
    std::cout << " axis: " << eaaFromRotation.axis() << std::endl;

    return 0;
}
