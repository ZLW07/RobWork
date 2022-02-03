import org.robwork.sdurw.*;
import org.robwork.sdurw.Q;
import static org.robwork.sdurw.sdurw.inverse;

public class ExFwdKinematicsDevice {
    public static void fwdKinematicsDevice(SerialDevicePtr sdevice, MovableFrame mframe, State state)
    {
        // get device base frame
        Frame base = sdevice.getBase();
        // get device end effector
        Frame end = sdevice.getEnd();
        // calculate base to end transform
        Transform3Dd bTe = sdevice.baseTend(state);
        // or just base to any frame
        Transform3Dd bTmf = sdevice.baseTframe(mframe, state);
        // get device name
        String sdevicename = sdevice.getName();
        // the degrees of freedom of this device
        long dof = sdevice.getDOF();
        // set the configuration of the device to zero
        sdevice.setQ( new Q((int)dof,0.0) , state );
    }
}