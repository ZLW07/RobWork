cmake_minimum_required(VERSION 3.10  )
# The name of the project. (EDIT THIS)
project(RWSimTutorialCollection)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION ##### Now set the RW/RWS root (edit this if necessary)
set(RobWorkSim_DIR "${ROOT}/../../../RobWorkSim/cmake")

# Include default settings for constructing a robwork dependent project
find_package(RobWorkSim REQUIRED)

# if you have additional libraries or include dirs then add them here
include_directories(${ROBWORKSIM_INCLUDE_DIRS})
link_directories(${ROBWORKSIM_LIBRARY_DIRS})

# And now we add any targets that we want
add_executable(SimLoop src/SimLoop.cpp)
target_link_libraries(SimLoop ${ROBWORKSIM_LIBRARIES})

