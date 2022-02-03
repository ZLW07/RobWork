import java.lang.String;

import org.robwork.*;
import org.robwork.sdurw.*;
import org.robwork.sdurw_simulation.*;
import org.robwork.sdurws.RobWorkStudioPtr;
import static org.robwork.sdurw_simulation.sdurw_simulation.ownedPtr;
import static org.robwork.sdurws.sdurws.getRobWorkStudioInstance;

public class SimulatedCameraExample {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");
        LoaderRW.load("sdurw_simulation");
        LoaderRWS.load();
        
        if (args.length != 1)
        	throw new Exception("Provide the path to RobWorkData as first argument.");
        
        final String WC_FILE =
                args[0] + "/scenes/SensorTestScene/SimpleWorkcell.xml";

        WorkCellPtr wc = WorkCellLoaderFactory.load(WC_FILE);
        if (wc.isNull())
            throw new Exception("WorkCell could not be loaded.");
        Frame camera = wc.findFrame("Camera");
        if (camera == null)
            throw new Exception("Camera frame could not be found.");
        PropertyMap properties = camera.getPropertyMap();
        if (!properties.has("Camera"))
            throw new Exception("Camera frame does not have Camera property.");
        String[] parameters = properties.getString("Camera").split(" ");
        Double fovy = Double.parseDouble(parameters[0]);
        Integer width = Integer.parseInt(parameters[1]);
        Integer height = Integer.parseInt(parameters[2]);
        System.out.print("Camera properties: fov " + fovy);
        System.out.println(" width " + width + " height " + height);

        RobWorkStudioPtr rwstudio = getRobWorkStudioInstance();
        rwstudio.postOpenWorkCell(WC_FILE);
        Thread.sleep(5000);
        SceneViewerPtr gldrawer = rwstudio.getView().getSceneViewer();
        GLFrameGrabberPtr framegrabber =
        		ownedPtr( new GLFrameGrabber(width,height,fovy) );
        framegrabber.init(gldrawer);
        SimulatedCamera simcam = new SimulatedCamera("SimulatedCamera",
        		fovy, camera, framegrabber.asFrameGrabberPtr());
        simcam.setFrameRate(100);
        simcam.initialize();
        simcam.start();
        simcam.acquire();

        final double DT = 0.001;
        Simulator.UpdateInfo info = new Simulator.UpdateInfo(DT);
        State state = wc.getDefaultState();
        int cnt = 0;
        Image img;
        while (!simcam.isImageReady()) {
            System.out.println("Image is not ready yet. Iteration " + cnt);
            simcam.update(info, state);
            cnt++;
        }
        img = simcam.getImage();
        img.saveAsPPM("Image1.ppm");
        simcam.acquire();
        while (!simcam.isImageReady()) {
            System.out.println("Image is not ready yet. Iteration " + cnt);
            simcam.update(info, state);
            cnt++;
        }
        System.out.println("Took " + cnt + " steps");
        img = simcam.getImage();
        System.out.print("Image: " + img.getWidth() + "x" + img.getHeight());
        System.out.print(" bits " + img.getBitsPerPixel());
        System.out.println(" channels " + img.getNrOfChannels());
        img.saveAsPPM("Image2.ppm");

        simcam.stop();
        rwstudio.postExit();
        Thread.sleep(1000);
    }
}
