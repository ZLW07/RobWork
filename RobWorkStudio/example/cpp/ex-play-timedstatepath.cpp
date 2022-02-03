#include <iostream>
#include <string>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

using namespace rws;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::trajectory;


int main (int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " /path/to/workcell.wc.xml /path/to/playback.rwplay [Option]" << std::endl;
        std::cout << "Option: "
                  << "-t     for test run" << std::endl;
        exit (1);
    }

    WorkCell::Ptr wc = WorkCellFactory::load(std::string(argv[1]));

    TimedStatePath t_path = PathLoader::loadTimedStatePath(wc,std::string(argv[2]));


    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->setWorkCell (wc);
        rwstudio->setTimedStatePath(t_path);

    }
    RWS_END()
}