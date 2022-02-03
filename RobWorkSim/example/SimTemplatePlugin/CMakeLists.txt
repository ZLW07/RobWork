#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.10  )

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# The name of the project.
PROJECT(SimTemplatePlugin)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
SET(RobWorkStudio_DIR "${ROOT}/../../../RobWorkStudio/cmake")
SET(RobWorkSim_DIR "${ROOT}/../../../RobWorkSim/cmake")

#Include RobWorkSim settings
FIND_PACKAGE(RobWorkStudio REQUIRED)
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

# if we want to use ui files add them here
SET(UIS_FILES SimTemplatePlugin.ui )

# to be able to include the generated ui header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}) 

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles SimTemplatePlugin.hpp)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles SimTemplatePlugin.hpp)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

SET(SrcFiles SimTemplatePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SimTemplatePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTemplatePlugin ${ROBWORKSIM_LIBRARIES} RWS::sdurws)

