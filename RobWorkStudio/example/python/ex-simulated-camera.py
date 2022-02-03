from sdurw import *
from sdurw_simulation import *
from sdurws import *
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise Exception("Provide the path to RobWorkData as first argument.")    
    WC_FILE = str(sys.argv[1]) + "/scenes/SensorTestScene/SimpleWorkcell.xml"

    wc = WorkCellLoaderFactory.load(WC_FILE)
    if wc.isNull():
        raise Exception("WorkCell could not be loaded")
    camera = wc.findFrame("Camera")
    if camera is None:
        raise Exception("Camera frame could not be found.")
    properties = camera.getPropertyMap()
    if not properties.has("Camera"):
        raise Exception("Camera frame does not have Camera property.")
    parameters = properties.getString("Camera").split(" ")
    fovy = float(parameters[0])
    width = int(parameters[1])
    height = int(parameters[2])
    print("Camera properties: fov " + str(fovy) + " width " + str(width) + " height " + str(height));

    rwstudio = getRobWorkStudioInstance();
    rwstudio.postOpenWorkCell(WC_FILE);
    sleep(5);
    gldrawer = rwstudio.getView().getSceneViewer();
    framegrabber = ownedPtr( GLFrameGrabber(width,height,fovy) );
    framegrabber.init(gldrawer);
    simcam = SimulatedCamera("SimulatedCamera", fovy, camera, framegrabber.asFrameGrabberPtr());
    simcam.setFrameRate(100);
    simcam.initialize();
    simcam.start();
    simcam.acquire();

    DT = 0.001;
    info = UpdateInfo(DT);
    state = wc.getDefaultState();
    cnt = 0;
    while not simcam.isImageReady():
        print("Image is not ready yet. Iteration " + str(cnt));
        simcam.update(info, state);
        cnt = cnt+1;
    img = simcam.getImage();
    img.saveAsPPM("Image1.ppm");
    simcam.acquire();
    while not simcam.isImageReady():
        print("Image is not ready yet. Iteration " + str(cnt));
        simcam.update(info, state);
        cnt = cnt+1;
    print("Took " + str(cnt) + " steps");
    img = simcam.getImage();
    print("Image: " + str(img.getWidth()) + "x" + str(img.getHeight()) + " bits " + str(img.getBitsPerPixel()) + " channels " + str(img.getNrOfChannels()));
    img.saveAsPPM("Image2.ppm");

    simcam.stop();
    rwstudio.postExit();
    sleep(1);