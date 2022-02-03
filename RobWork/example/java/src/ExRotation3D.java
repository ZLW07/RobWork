import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurw.*;

public class ExRotation3D {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        Rotation3Dd rotd = new Rotation3Dd(1,0,0,0,0,-1,0,1,0);
        Rotation3Df rotf = new Rotation3Df(1,0,0,0,0,-1,0,1,0);

        System.out.println("Rotation double:");
        System.out.println(rotd);
        System.out.println("Rotation float:");
        System.out.println(rotf);
        System.out.println("Rotation inverse:");
        System.out.println(rotd.inverse());
        System.out.println("Identity:");
        System.out.println(rotd.multiply(inverse(rotd)));
    }
}
