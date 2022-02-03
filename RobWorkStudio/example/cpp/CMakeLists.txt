cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()

find_package(RobWorkStudio REQUIRED PATHS "$ENV{RWS_ROOT}" "../../cmake")

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC RWS::sdurws RWS::sdurws_robworkstudioapp RW::sdurw_simulation)
    target_include_directories(${_name} PUBLIC ${ROBWORKSTUDIO_INCLUDE_DIRS})
endmacro()

macro(rw_build_lib _name)
    # And now we add any targets that we want
    add_library(${_name} ${_name}.cpp)
    target_link_libraries(${_name} PUBLIC RWS::sdurws RWS::robworkstudioapp RW::sdurw_simulation)
    target_include_directories(${_name} PUBLIC ${ROBWORKSTUDIO_INCLUDE_DIRS})
endmacro()

rw_build(ex-simulated-camera)
rw_build(ex-load-workcell)
#rw_build(ex-play-timedstatepath)

foreach(obj ${ROBWORKSTUDIO_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORKSTUDIO_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
