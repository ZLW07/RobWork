import java.util.Iterator;

import org.robwork.sdurw.*;

public class ExPrintDevices {

    public static void printDeviceNames(WorkCellPtr workcell)
    {
        System.out.println("Workcell " + workcell.getName() + " contains devices:");
        
        // Method 1
        DevicePtrVector devices = workcell.getDevices();
        DevicePtr[] array = new DevicePtr[devices.size()];
        array = devices.toArray(array);
        for(DevicePtr device : array) {
            System.out.println("- " + device.getName());
        }
        
        // Method 2
        Iterator<DevicePtr> iterator = devices.iterator();
        while(iterator.hasNext()) {
            DevicePtr device = iterator.next();
            System.out.println("* " + device.getName());
        }
        
        // Method 3
        for (int i = 0; i < devices.size(); i++) {
            System.out.println(". " + devices.get(i).getName());
        }
    }

}
