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

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# The name of the project.
project(PluginUIApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE false)

# Now set the RobWorkSim directories (edit this if necessary)
set(RWSIM_ROOT "${ROOT}/../../../RobWorkSim")
set(RobWorkSim_DIR "${RWSIM_ROOT}/cmake")

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Include RobWorkSim settings

message(" ${RWSIM_ROOT} --")
find_package(RobWorkSim REQUIRED)
include_directories(${ROBWORKSIM_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${ROBWORKSIM_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

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
set(UIS_FILES SimTaskVisPlugin.ui)

# to be able to include the generated ui header files
include_directories(${CMAKE_CURRENT_BINARY_DIR})

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles SimTaskVisPlugin.hpp)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles SimTaskVisPlugin.hpp)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()
set(SrcFiles SimTaskVisPlugin.cpp ${UIS_OUT_H})

# The shared library to build:
add_library(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(SimTaskVisPlugin ${ROBWORKSIM_LIBRARIES} RWS::sdurws)
