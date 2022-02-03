#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/ProximityData.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using rw::core::ownedPtr;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using rw::math::Q;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using rw::trajectory::QPath;
using rwlibs::pathplanners::RRTPlanner;
using rwlibs::proximitystrategies::ProximityStrategyFactory;

#define WC_FILE "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <path/to/RobWorkData>" << std::endl;
        exit (1);
    }
    const std::string path = argv[1];

    const WorkCell::Ptr wc = WorkCellLoader::Factory::load (path + WC_FILE);
    if (wc.isNull ())
        RW_THROW ("WorkCell could not be loaded.");
    const Device::Ptr gantry = wc->findDevice ("Gantry");
    const Device::Ptr pa10   = wc->findDevice ("PA10");
    if (gantry.isNull ())
        RW_THROW ("Gantry device could not be found.");
    if (pa10.isNull ())
        RW_THROW ("PA10 device could not be found.");

    const State defState     = wc->getDefaultState ();
    const Device::Ptr device = ownedPtr (new CompositeDevice (
        gantry->getBase (), wc->getDevices (), pa10->getEnd (), "Composite", defState));

    const CollisionStrategy::Ptr cdstrategy =
        ProximityStrategyFactory::makeCollisionStrategy ("PQP");
    if (cdstrategy.isNull ())
        RW_THROW ("PQP Collision Strategy could not be found.");
    const CollisionDetector::Ptr collisionDetector =
        ownedPtr (new CollisionDetector (wc, cdstrategy));
    const PlannerConstraint con    = PlannerConstraint::make (collisionDetector, device, defState);
    const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (con, device);

    const Q beg (9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0);
    const Q end (9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0);

    ProximityData pdata;
    State state = defState;
    device->setQ (beg, state);
    if (collisionDetector->inCollision (state, pdata))
        RW_THROW ("Initial configuration in collision! can not plan a path.");
    device->setQ (end, state);
    if (collisionDetector->inCollision (state, pdata))
        RW_THROW ("Final configuration in collision! can not plan a path.");

    QPath result;
    if (planner->query (beg, end, result)) {
        std::cout << "Planned path with " << result.size ();
        std::cout << " configurations" << std::endl;
    }

    return 0;
}
