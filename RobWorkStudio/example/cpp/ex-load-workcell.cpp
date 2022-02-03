#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <thread>

using rw::common::TimerUtil;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;

int main (int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " /path/to/robworkdata [Option]" << std::endl;
        std::cout << "Option: "
                  << "-t     for test run" << std::endl;
        exit (1);
    }

    const std::string WC_FILE =
        std::string (argv[1]) + "/scenes/SinglePA10Demo/SinglePA10Demo.wc.xml";

    rws::RobWorkStudioApp rwsApp ("");
    RWS_START (rwsApp)
    {
        rws::RobWorkStudio* rwstudio = rwsApp.getRobWorkStudio ();
        rwstudio->setWorkCell (WC_FILE);

        while (rwsApp.isRunning ()) {
            TimerUtil::sleepMs (10);    // Check if running every 10 ms
            if (argc == 3 && std::string (argv[2]) == std::string ("-t")) {    // Quit if test run
                TimerUtil::sleepMs (4000);    // wait for full startup
                rwsApp.close ();    // Close RWS
            }
        }
        rwsApp.close ();    // Close rws if running
        TimerUtil::sleepMs (10000);    // wait for full startup
    }
    RWS_END ();

    return 0;
}
