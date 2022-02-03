#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

#include <iostream>

using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

#define WC_FILE "/devices/serialdev/UR10e_2018/UR10e.xml"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <path/to/RobWorkData>" << std::endl;
        exit(1);
    }
    const std::string path = argv[1];

    const WorkCell::Ptr wc = WorkCellLoader::Factory::load(path + WC_FILE);
    if (wc.isNull())
        RW_THROW("WorkCell could not be loaded.");
    const SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR10e");
    if (device.isNull())
        RW_THROW("UR10e device could not be found.");

    const State state = wc->getDefaultState();
    const ClosedFormIKSolverUR solver(device, state);

    const Transform3D<> Tdesired(Vector3D<>(0.2, -0.2, 0.5),
            EAA<>(0, Pi, 0).toRotation3D());
    const std::vector<Q> solutions = solver.solve(Tdesired, state);

    std::cout << "Inverse Kinematics for " << device->getName() << "." << std::endl;
    std::cout << " Base frame: " << device->getBase()->getName() << std::endl;
    std::cout << " End/TCP frame: " << solver.getTCP()->getName() << std::endl;
    std::cout << " Target Transform: " << Tdesired << std::endl;
    std::cout << "Found " << solutions.size() << " solutions." << std::endl;
    for(std::size_t i = 0; i < solutions.size(); i++) {
        std::cout << " " << solutions[i] << std::endl;
    }

    return 0;
}
