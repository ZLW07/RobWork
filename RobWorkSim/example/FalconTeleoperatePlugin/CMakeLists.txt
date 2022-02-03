# ######################################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into libraries - the
# plugins contain code for multiple RobWorkStudio plugins - the test for testing - the example dir for illustrativ
# examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies to external projects
# ######################################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10  )

# The name of the project.
project(FalconControl)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
set(RobWorkStudio_DIR "$ENV{RW_ROOT}/../RobWorkStudio/cmake")
set(RobWorkSim_DIR "$ENV{RW_ROOT}/../RobWorkSim/cmake")

# Include RobWorkSim settings
find_package(RobWorkStudio REQUIRED)
find_package(RobWorkSim REQUIRED)
include_directories(${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS})
link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
endif()

# if we want to use ui files add them here
set(UIS_FILES src/FalconControl.ui)

rws_qt4_wrap_ui(UIS_OUT_H ${UIS_FILES})

qt4_wrap_cpp(MocSrcFiles src/FalconControl.hpp)
# Rcc the files:

qt4_add_resources(RccSrcFiles resources.qrc)
set(SrcFiles src/FalconControl.cpp src/FalconInterface.cpp src/IO.cpp ${UIS_OUT_H})

# The shared library to build:
add_library(FalconControl MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(FalconControl ${ROBWORKSIM_LIBRARIES} RWS::sdurws ${ROBWORK_LIBRARIES}
                      libnifalcon.so)
