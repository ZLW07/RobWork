import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurwConstants.*;

public class ExRPY {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        RPYd rpy = new RPYd(Pi,Pi/2,0);
        System.out.println("RPY: " + rpy);
        Rotation3Dd rotationFromRPY = rpy.toRotation3D();
        System.out.println("Rotation from RPY: " + rotationFromRPY);

        Rotation3Dd rot = new Rotation3Dd(-1,0,0,0,0,1,0,1,0);
        System.out.println("Rotation: " + rot);
        RPYd rpyFromRotation = new RPYd(rot);
        System.out.println("RPY from Rotation: " + rpyFromRotation);
    }
}
