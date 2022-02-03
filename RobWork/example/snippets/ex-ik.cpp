#include <rw/invkin/JacobianIKSolver.hpp>

using rw::invkin::JacobianIKSolver;
using rw::kinematics::State;
using namespace rw::math;
using rw::models::Device;

void inverseKinematics(rw::core::Ptr<Device> device, const State& state, const Transform3D<>& target)
{
    JacobianIKSolver solver(device.cptr(), state);
	std::vector<Q> solutions = solver.solve(target, state);
	for(Q q : solutions) {
		std::cout<<"Solution = "<<q<<std::endl;
	}
}
