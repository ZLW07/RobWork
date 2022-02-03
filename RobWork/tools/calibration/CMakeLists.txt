# ##############################################################################
# Example based on simple opengl gui that use some RobWork functionality. - the
# src should contain code for putting into libraries
#
# Use config.cmake to change default settings and edit this file for adding
# additional dependencies to external projects
# ##############################################################################

# Test CMake version
cmake_minimum_required(VERSION 3.10  FATAL_ERROR )
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD)
endif()

# The name of the project.
project(sdurw_calibration_tools)

# optionally: point the find package in the direction of the sdurw.
set(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
find_package(ROBWORK REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

set(CALIBRATION_DEPENDENCIES
    sdurw
    sdurw_assembly
    sdurw_algorithms
    sdurw_calibration
    sdurw_opengl
    sdurw_pathoptimization
    sdurw_pathplanners
    sdurw_proximitystrategies
    sdurw_simulation
    sdurw_task)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)

add_executable(sdurw_calibration-tool CalibrationTool.cpp)
target_link_libraries(sdurw_calibration-tool ${ROBWORK_LIBRARIES})
add_dependencies(sdurw_calibration-tool ${CALIBRATION_DEPENDENCIES})

add_executable(sdurw_calibration-mtool MeasurementTool.cpp)
target_link_libraries(sdurw_calibration-mtool ${ROBWORK_LIBRARIES})
add_dependencies(sdurw_calibration-mtool ${CALIBRATION_DEPENDENCIES})

add_executable(sdurw_calibration-verification-tool
               CalibrationVerificationTool.cpp)
target_link_libraries(sdurw_calibration-verification-tool ${ROBWORK_LIBRARIES})
add_dependencies(sdurw_calibration-verification-tool
                 ${CALIBRATION_DEPENDENCIES})
