import java.lang.String;
import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurw.ownedPtr;
import org.robwork.sdurw_pathplanners.RRTPlanner;
import org.robwork.sdurw_proximitystrategies.ProximityStrategyFactory;

public class ExMotionPlanning {
    public static final String WC_FILE =
            "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml";

    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");
        LoaderRW.load("sdurw_pathplanners");
        LoaderRW.load("sdurw_proximitystrategies");

        if (args.length != 1) {
            System.out.print("Usage: java " + ExMotionPlanning.class.getSimpleName());
            System.out.println(" <path/to/RobWorkData>");
            System.exit(1);
        }

        WorkCellPtr wc = WorkCellLoaderFactory.load(args[0] + WC_FILE);
        if (wc.isNull())
            throw new Exception("WorkCell could not be loaded.");
        DevicePtr gantry = wc.findDevice("Gantry");
        DevicePtr pa10 = wc.findDevice("PA10");
        if (gantry.isNull())
            throw new Exception("Gantry device could not be found.");
        if (pa10.isNull())
            throw new Exception("PA10 device could not be found.");

        State defState = wc.getDefaultState();
        CompositeDevicePtr device = ownedPtr(new CompositeDevice(gantry.getBase(),
                wc.getDevices(), pa10.getEnd(), "Composite", defState));

        CollisionStrategyPtr cdstrategy =
                ProximityStrategyFactory.makeCollisionStrategy("PQP");
        if (cdstrategy.isNull())
            throw new Exception("PQP Collision Strategy could not be found.");
        CollisionDetectorPtr collisionDetector = ownedPtr(
                new CollisionDetector(wc, cdstrategy));
        PlannerConstraint con = PlannerConstraint.make(collisionDetector,
                device.asDeviceCPtr(), defState);
        QToQPlannerPtr planner = RRTPlanner.makeQToQPlanner(con, device.asDevicePtr());
        State state = wc.getDefaultState();

        final Q beg = new Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0);
        final Q end = new Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0);

        ProximityData pdata = new ProximityData();
        device.setQ(beg, state);
        if (collisionDetector.inCollision(state, pdata))
            throw new Exception("Initial configuration in collision!");
        device.setQ(end, state);
        if (collisionDetector.inCollision(state, pdata))
            throw new Exception("Final configuration in collision!");

        PathQ result = new PathQ();
        if (planner.query(beg, end, result)) {
            System.out.print("Planned path with " + result.size());
            System.out.println(" configurations");
        }
    }
}
