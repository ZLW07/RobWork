import org.robwork.sdurw.*;

public class ExFindFromWorkCell {
    public static void findFromWorkCell(WorkCellPtr wc)
    {
        // get the default state
        State state = wc.getDefaultState();
        Frame worldFrame = wc.getWorldFrame();
        // find a frame by name, remember NULL is a valid return
        Frame frame = wc.findFrame("FixedFrameName");
        // find a frame by name, but with a specific frame type
        FixedFrame fframe = wc.findFixedFrame("FixedFrameName");
        MovableFrame mframe = wc.findMovableFrame("MovableFrameName");
        // find a device by name
        DevicePtr device = wc.findDevice("SerialDeviceName");
        SerialDevicePtr sdevice = wc.findSerialDevice("SerialDeviceName");
    }
}
