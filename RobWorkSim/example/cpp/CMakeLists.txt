cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWS_ROOT "$ENV{RWS_ROOT}")
set(RWSIM_ROOT "$ENV{RWSIM_ROOT}")

find_package(RobWorkSim REQUIRED PATHS "${RWSIM_ROOT}" "../../cmake")

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC ${ROBWORKSIM_LIBRARIES})
    target_include_directories(${_name} PUBLIC ${ROBWORKSIM_INCLUDE_DIRS})
endmacro()

macro(rw_build_lib _name)
    # And now we add any targets that we want
    add_library(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC ${ROBWORKSIM_LIBRARIES})
    target_include_directories(${_name} PUBLIC  ${ROBWORKSIM_INCLUDE_DIRS})
endmacro()

rw_build(ex-ode-simloop)

foreach(obj ${ROBWORKSIM_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSIM_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSIM_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
