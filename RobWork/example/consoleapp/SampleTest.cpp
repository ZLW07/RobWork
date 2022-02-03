#include <rw/core/Log.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using rw::core::Log;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;

int main(int argc, char** argv)
{
    if (argc != 2) {
        Log::errorLog() << "Usage: " << argv[0] << " <workcell>" << std::endl;
        return 1;
    }

    const std::string file = argv[1];
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load(file);
    if (workcell.isNull()) {
        Log::errorLog() << "WorkCell could not be loaded." << std::endl;
        return 1;
    }

    Log::infoLog() << "Workcell " << workcell->getName();
    Log::infoLog() << " successfully loaded." << std::endl;
    return 0;
}
