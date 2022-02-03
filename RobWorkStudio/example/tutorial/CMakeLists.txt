cmake_minimum_required(VERSION 3.10  )
# The name of the project.
project(SamplePluginApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
set(RW_ROOT "${ROOT}/../../../RobWork")
set(RWS_ROOT "${ROOT}/../../../RobWorkStudio")

# We use the settings that robwork studio uses
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(RobWorkStudio_DIR "${RWS_ROOT}/cmake")
find_package(RobWorkStudio)

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

# ########################   From here we add the plugins

if(DEFINED Qt6Core_VERSION)
    qt_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePluginApp)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_cpp(MocSrcFiles SamplePlugin.hpp TARGET SamplePluginApp)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

# The shared library to build:
add_library(SamplePluginApp MODULE SamplePlugin.cpp ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(SamplePluginApp RWS::sdurws ${ROBWORK_LIBRARIES})
target_include_directories(SamplePluginApp PUBLIC ${ROBWORK_INCLUDE_DIRS})

foreach(obj ${ROBWORKSTUDIO_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
