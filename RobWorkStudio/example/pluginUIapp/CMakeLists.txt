# Test CMake version
cmake_minimum_required(VERSION 3.10  )

# The name of the project.
project(PluginUIApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
if(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
  file(TO_CMAKE_PATH "$ENV{RW_ROOT}" RW_ROOT)
  file(TO_CMAKE_PATH "$ENV{RWS_ROOT}" RWSTUDIO_ROOT)
else()
  set(RW_ROOT "${ROOT}/../../../RobWork")
  set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
endif()

# If not the -DCMAKE_BUILD_TYPE option is given when running CMake, use Release as default.
if( NOT DEFINED CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Include default settings for constructing a RobWork dependent project
set(RobWork_DIR ${RW_ROOT}/cmake)
find_package(RobWork REQUIRED)
message("-- RobWork found: " ${RW_ROOT})

# Include default settings for constructing a RobWorkStudio dependent project
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
find_package(RobWorkStudio REQUIRED)
message("-- RobWorkStudio found: " ${RWSTUDIO_ROOT})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add the source files to build
set(SrcFiles SamplePlugin.cpp resources.qrc)

# The shared library to build:
add_library(PluginUIApp MODULE ${SrcFiles})
target_link_libraries(PluginUIApp PUBLIC RWS::sdurws RW::sdurw)

if(CMAKE_VERSION VERSION_LESS 3.9)
    # Required in CMake < 3.9 to find generated ui_SamplePlugin.h file 
    target_include_directories(PluginUIApp PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
endif()

message("-- " ${PROJECT_NAME} " done!")
