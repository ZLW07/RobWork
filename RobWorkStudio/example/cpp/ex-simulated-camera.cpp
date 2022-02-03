#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

using namespace rw::core;
using namespace rw::common;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::simulation;
using namespace rws;

int main (int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << "/path/to/robworkdata" << std::endl;
        exit (1);
    }

    static const std::string WC_FILE =
        std::string (argv[1]) + "/scenes/SensorTestScene/SimpleWorkcell.xml";
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load (WC_FILE);
    if (wc.isNull ())
        RW_THROW ("WorkCell could not be loaded.");
    Frame* const camera = wc->findFrame ("Camera");
    if (camera == nullptr)
        RW_THROW ("Camera frame could not be found.");
    const PropertyMap& properties = camera->getPropertyMap ();
    if (!properties.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");
    const std::string parameters = properties.get< std::string > ("Camera");
    std::istringstream iss (parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    std::cout << "Camera properties: fov " << fovy << " width " << width << " height " << height
              << std::endl;

    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell (WC_FILE);
        TimerUtil::sleepMs (5000);

        const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const GLFrameGrabber::Ptr framegrabber =
            ownedPtr (new GLFrameGrabber (width, height, fovy));
        framegrabber->init (gldrawer);
        SimulatedCamera::Ptr simcam =
            ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy, camera, framegrabber));
        simcam->setFrameRate (100);
        simcam->initialize ();
        simcam->start ();
        simcam->acquire ();

        static const double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState ();
        int cnt     = 0;
        const Image* img;
        while (!simcam->isImageReady ()) {
            std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam->update (info, state);
            cnt++;
        }
        img = simcam->getImage ();
        img->saveAsPPM ("Image1.ppm");
        simcam->acquire ();
        while (!simcam->isImageReady ()) {
            std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam->update (info, state);
            cnt++;
        }
        std::cout << "Took " << cnt << " steps" << std::endl;
        img = simcam->getImage ();
        std::cout << "Image: " << img->getWidth () << "x" << img->getHeight () << " bits "
                  << img->getBitsPerPixel () << " channels " << img->getNrOfChannels ()
                  << std::endl;
        img->saveAsPPM ("Image2.ppm");

        simcam->stop ();
        app.close ();
    }
    RWS_END ()

    return 0;
}
