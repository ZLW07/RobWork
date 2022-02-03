import sdurw_models
from sdurw import *
from sdurw_pathplanners import RRTPlanner
from sdurw_proximitystrategies import ProximityStrategyFactory
import sys

WC_FILE = "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 " + sys.argv[0] + " <path/to/RobWorkData>")
        sys.exit(1)

    wc = WorkCellLoaderFactory.load(sys.argv[1] + WC_FILE)
    if wc.isNull():
        raise Exception("WorkCell could not be loaded")
    gantry = wc.findDevice("Gantry")
    pa10 = wc.findDevice("PA10")
    if gantry.isNull():
        raise Exception("Gantry device could not be found.")
    if pa10.isNull():
        raise Exception("PA10 device could not be found.")

    defState = wc.getDefaultState()
    device = sdurw_models.ownedPtr(CompositeDevice(gantry.getBase(), wc.getDevices(), pa10.getEnd(), "Composite", defState))

    cdstrategy = ProximityStrategyFactory.makeCollisionStrategy("PQP")
    if cdstrategy.isNull():
        raise Exception("PQP Collision Strategy could not be found.")
    print(device.cptr())
    collisionDetector = sdurw_proximity.ownedPtr(CollisionDetector(wc, cdstrategy))
    con = PlannerConstraint.make(collisionDetector, device.cptr(), defState)
    planner = RRTPlanner.makeQToQPlanner(con, device.asDevicePtr())

    beg = Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0)
    end = Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0)

    pdata = ProximityData()
    state = defState
    device.setQ(beg, state)
    if collisionDetector.inCollision(state, pdata):
        raise Exception("Initial configuration in collision!")
    device.setQ(end, state)
    if collisionDetector.inCollision(state, pdata):
        raise Exception("Final configuration in collision!")

    result = PathQ()
    if planner.query(beg, end, result):
        print("Planned path with " + str(result.size()) + " configurations")
