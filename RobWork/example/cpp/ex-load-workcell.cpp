#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>" << std::endl;
        return 1;
    }

    const std::string file = argv[1];
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load(file);
    if (workcell.isNull()) {
        std::cout << "WorkCell could not be loaded." << std::endl;
        return 1;
    }

    std::cout << "Workcell " << workcell->getName();
    std::cout << " successfully loaded." << std::endl;
    return 0;
}
