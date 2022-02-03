import java.lang.String;
import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurwConstants.Pi;


public class ExInvKin {
    public static final String WC_FILE =
            "/devices/serialdev/UR10e_2018/UR10e.xml";

    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        if (args.length != 1) {
            System.out.print("Usage: java " + ExInvKin.class.getSimpleName());
            System.out.println(" <path/to/RobWorkData>");
            System.exit(1);
        }

        WorkCellPtr wc = WorkCellLoaderFactory.load(args[0] + WC_FILE);
        if (wc.isNull())
            throw new Exception("WorkCell could not be loaded.");
        SerialDevicePtr device = wc.findSerialDevice("UR10e");
        if (device.isNull())
            throw new Exception("UR10e device could not be found.");

        State state = wc.getDefaultState();
        ClosedFormIKSolverUR solver = new ClosedFormIKSolverUR(
                device.asSerialDeviceCPtr(), state);

        final Transform3Dd Tdesired = new Transform3Dd(new Vector3Dd(0.2, -0.2, 0.5),
                (new EAAd(0, Pi, 0)).toRotation3D());
        final QVector solutions = solver.solve(Tdesired, state);

        System.out.println("Inverse Kinematics for " + device.getName() + ".");
        System.out.println(" Base frame: " + device.getBase().getName());
        System.out.println(" End/TCP frame: " + solver.getTCP().getName());
        System.out.println(" Target Transform: " + Tdesired.toString());
        System.out.println("Found " + solutions.size() + " solutions.");
        for(int i = 0; i < solutions.size(); i++) {
            System.out.println(" " + solutions.get(i).toString());
        }
    }
}
