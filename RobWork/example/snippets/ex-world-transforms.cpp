#include <rw/kinematics/FKTable.hpp>

#include <vector>

using rw::math::Transform3D;
using namespace rw::kinematics;

std::vector<Transform3D<> > worldTransforms(
    const std::vector<Frame*>& frames, const State& state)
{
    FKTable fk(state);

    std::vector<Transform3D<> > result;
    for(Frame* f : frames) {
        result.push_back(fk.get(*f));
    }
    return result;
}
