import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import java.lang.Math;

public class ExQuaternion {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        Quaterniond quat = new Quaterniond(Math.sqrt(2)/2,Math.sqrt(2)/2,0,0);
        System.out.println("Quaternion: " + quat);
        Rotation3Dd rotationFromQuat = quat.toRotation3D();
        System.out.println("Rotation from Quaternion: " + rotationFromQuat);

        Rotation3Dd rot = new Rotation3Dd(-1,0,0,0,0,1,0,1,0);
        System.out.println("Rotation: " + rot);
        Quaterniond quatFromRotation = new Quaterniond(rot);
        System.out.println("Quaternion from Rotation: " + quatFromRotation);
    }
}
