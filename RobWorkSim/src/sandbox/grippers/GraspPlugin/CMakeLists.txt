cmake_minimum_required(VERSION 3.10  )

# The name of the project.
project(GraspPlugin)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(RW_ROOT "$ENV{RW_ROOT}")
set(RWS_ROOT "$ENV{RWS_ROOT}")
set(RWSIM_ROOT "$ENV{RWSIM_ROOT}")

set(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWS_ROOT}/cmake)
set(RobWorkSim_DIR ${RWSIM_ROOT}/cmake)

set(CMAKE_MODULE_PATH ${RWHW_ROOT}/build ${CMAKE_MODULE_PATH})

# We use the settings that robwork studio uses
set(CMAKE_BUILD_TYPE Release)

# Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(RobWorkSim REQUIRED)
find_package(ODE)
include_directories(${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})
# message("!!!!!!!!" ${ROBWORKSIM_LIBRARY_DIRS}) Set the output dir for generated libraries and binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)

# ######################################################################################################################
# From here we add the plugins
set(
    SrcFiles
    TaskGenerator.cpp
    csgjs.cpp
    CSGConvert.cpp
    CSGModel.cpp
    JawPrimitive.cpp
    Gripper.cpp
    GripperXMLLoader.cpp
    GripperTaskSimulator.cpp
    TaskDescription.cpp
    XMLHelpers.cpp
    SurfaceSample.cpp
)

set(UisFiles GraspPlugin.ui DesignDialog.ui TaskDialog.ui)

set(MocSrcFiles)
qt5_wrap_cpp(MocSrcFiles GraspPlugin.hpp DesignDialog.hpp TaskDialog.hpp)
set(RccSrcFiles)
qt5_add_resources(RccSrcFiles resources.qrc)
qt5_wrap_ui(UiHeaders ${UisFiles})

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(
        doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif(DOXYGEN_FOUND)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")

# MESSAGE(${ROBWORKSIM_LIBRARIES}) MESSAGE("!" ${ODE_LIBRARIES}) The shared library to build
add_library(GraspLibrary ${SrcFiles})
add_library(
    GraspPlugin MODULE
    GraspPlugin.cpp
    DesignDialog.cpp
    TaskDialog.cpp
    ${MocSrcFiles}
    ${RccSrcFiles}
    ${UiHeaders}
)
# add_executable( test test.cpp csgjs.cpp CSGConvert.cpp CSGModel.cpp JawPrimitive.cpp)
add_executable(create-gripper-xml createGripperXML.cpp)
add_executable(evaluate-gripper evaluateGripper.cpp)
# add_executable(simulate-perturbed-tasks simulatePerturbedTasks.cpp) add_executable(evaluate-perturbed-tasks
# evaluatePerturbedTasks.cpp)
add_executable(generate-surface-samples generateSurfaceSamples.cpp)
# add_executable( create-gripper-db createGripperDB.cpp)

target_link_libraries(
    GraspLibrary
    -lGL
    -lGLU
    -lglut
    -lm
    RWS::sdurws
    ${ROBWORK_LIBRARIES}
    ${ROBWORKSIM_LIBRARIES}
)
target_link_libraries(GraspPlugin GraspLibrary)
target_link_libraries(create-gripper-xml GraspLibrary)
target_link_libraries(
    evaluate-gripper
    ode_plugin.rwplugin
    GraspLibrary
    RWS::sdurws
    ${ROBWORK_LIBRARIES}
    ${ROBWORKSIM_LIBRARIES}
)

target_link_libraries(generate-surface-samples GraspLibrary ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES}
                      ${ROBWORKSIM_LIBRARIES})
