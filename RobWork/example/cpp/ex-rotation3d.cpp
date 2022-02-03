#include <rw/math/Rotation3D.hpp>

using rw::math::Rotation3D;

int main(int argc, char** argv) {
    Rotation3D<> rotd = Rotation3D<>(1,0,0,0,0,-1,0,1,0);
    Rotation3D<float> rotf = Rotation3D<float>(1,0,0,0,0,-1,0,1,0);

    std::cout << "Rotation double:" << std::endl << rotd << std::endl;
    std::cout << "Rotation float:" << std::endl << rotf << std::endl;
    std::cout << "Rotation inverse:" << std::endl << inverse(rotd) << std::endl;
    std::cout << "Identity:" << std::endl << rotd*inverse(rotd) << std::endl;

    return 0;
}
