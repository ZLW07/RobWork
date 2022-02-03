#include <rw/kinematics/FKRange.hpp>
#include <rw/kinematics/State.hpp>

#include <vector>

using rw::math::Transform3D;
using namespace rw::kinematics;

std::vector<Transform3D<> > frameToFrameTransforms(
    const Frame& a,
    const Frame& b,
    const State& tree_structure,
    const std::vector<State>& states)
{
    FKRange fk(&a, &b, tree_structure);

    std::vector<Transform3D<> > result;
    for(const State& state : states) {
        result.push_back(fk.get(state));
    }
    return result;
}
