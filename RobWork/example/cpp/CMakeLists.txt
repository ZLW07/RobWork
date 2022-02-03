cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

project(ExampleSnippets CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()

find_package(RobWork REQUIRED PATHS "${RW_ROOT}" "$ENV{RW_ROOT}" "../../cmake")

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} ${ROBWORK_LIBRARIES})
endmacro()

macro(rw_build_lib _name)
    # And now we add any targets that we want
    add_library(${_name} ${_name}.cpp)
    target_link_libraries(${_name} ${ROBWORK_LIBRARIES})
endmacro()

rw_build(ex-eaa)

rw_build(ex-invkin)
rw_build(ex-load-workcell)
rw_build(ex-motionplanning)
rw_build(ex-quaternion)
rw_build(ex-rotation3d)
rw_build(ex-rpy)
rw_build(ex-collisions)

rw_build_lib(ex_world-transforms)
rw_build_lib(ex_find-from-workcell)
rw_build_lib(ex_frame-to-frame-transform)
rw_build_lib(ex_frame-to-frame-transforms)
rw_build_lib(ex_fwd-kinematics-device)
rw_build_lib(ex_fwd-kinematics)
rw_build_lib(ex_print-devices)
rw_build_lib(ex_print-kinematic-tree)

foreach(obj ${ROBWORK_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
