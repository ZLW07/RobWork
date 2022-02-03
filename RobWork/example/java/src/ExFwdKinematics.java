import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurw.inverse;

public class ExFwdKinematics {
    public static void fwdKinematics(SerialDevicePtr sdevice,
            Frame frame, MovableFrame mframe, State state)
    {
        // calculate the transform from one frame to another
        Transform3Dd fTmf = Kinematics.frameTframe(frame, mframe, state);
        // calculate the transform from world to frame
        Transform3Dd wTmf = Kinematics.worldTframe( mframe, state );
        // we can find the world to frame transform by a little jogling
        Transform3Dd wTf = wTmf.multiply(inverse(fTmf));
        // test if frame is a dynamic attachable frame
        if( Kinematics.isDAF( mframe ) ){
           // attach mframe to end of serial device
           Kinematics.gripFrame(mframe, sdevice.getEnd(), state);
        }
    }
}