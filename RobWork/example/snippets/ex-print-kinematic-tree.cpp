#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>

#include <string>

using namespace rw::kinematics;
using rw::models::WorkCell;
using rw::math::Transform3D;

void printKinematicTree(
    Frame& frame,
    const State& state,
    const Transform3D<>& parentTransform,
    int level)
{
    const Transform3D<> transform = parentTransform * frame.getTransform(state);

    std::cout
        << std::string(level, ' ')
        << frame.getName()
        << " at "
        << transform.P()
        << std::endl;

    for(Frame::Ptr& child: frame.getChildrenList(state)) {
        printKinematicTree(*child.get(), state, transform, level + 1);
    }

    /*for(Frame& child: frame.getChildrenList(state)) {
        printKinematicTree(child, state, transform, level + 1);
    }*/
}

void printDefaultWorkCellStructure(const WorkCell& workcell)
{
    std::cout << workcell << std::endl;
    printKinematicTree(
        *workcell.getWorldFrame(),
        workcell.getDefaultState(),
        Transform3D<>::identity(),
        0);
}
