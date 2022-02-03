# ##################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into
# libraries - the plugins contain code for multiple RobWorkStudio plugins - the test for testing -
# the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies
# to external projects
# ##################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10  )

# The name of the project.
project(UDPKinPlugin)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# set(CMAKE_VERBOSE_MAKEFILE True)

set(RW_ROOT "${ROOT}/../../../RobWork")
set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
set(RobWork_DIR "${RW_ROOT}/cmake")
set(RobWorkStudio_DIR "${RWSTUDIO_ROOT}/cmake")

# We use the settings that robwork studio uses
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(
    Boost REQUIRED
    COMPONENTS regex
)

# if we want to use ui files add them here
set(UIS_FILES SamplePlugin.ui)

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET UDPKinPlugin)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET UDPKinPlugin)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

set(SrcFiles SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
add_library(UDPKinPlugin MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(UDPKinPlugin RWS::sdurws RW::sdurw ${Boost_LIBRARIES})
target_include_directories(
    UDPKinPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
