# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.17)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget RWSIM::sdurwsim RWSIM::sdurwsim_gui RWSIM::sdurwsim_ode RWSIM::sdurwsim_bullet RWSIM::sdurwsim_test)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target RWSIM::sdurwsim
add_library(RWSIM::sdurwsim SHARED IMPORTED)

set_target_properties(RWSIM::sdurwsim PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zw/CLionProjects/RobWork/RobWorkSim/src"
  INTERFACE_LINK_LIBRARIES "RW::sdurw_algorithms;RW::sdurw_assembly;RW::sdurw_opengl;RW::sdurw_proximitystrategies;RW::sdurw_pathplanners;RW::sdurw_simulation;RW::sdurw_task;RW::sdurw_core;RW::sdurw_math;RW::sdurw;/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so"
)

# Create imported target RWSIM::sdurwsim_gui
add_library(RWSIM::sdurwsim_gui SHARED IMPORTED)

set_target_properties(RWSIM::sdurwsim_gui PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zw/CLionProjects/RobWork/RobWorkSim/src"
  INTERFACE_LINK_LIBRARIES "RWSIM::sdurwsim;RWS::sdurws;Qt5::Core;Qt5::Gui;Qt5::Widgets;Qt5::OpenGL"
)

# Create imported target RWSIM::sdurwsim_ode
add_library(RWSIM::sdurwsim_ode SHARED IMPORTED)

set_target_properties(RWSIM::sdurwsim_ode PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zw/CLionProjects/RobWork/RobWorkSim/src;/usr/include"
  INTERFACE_LINK_LIBRARIES "RWSIM::sdurwsim;/usr/lib/x86_64-linux-gnu/libode.so;RW::sdurw;/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so"
)

# Create imported target RWSIM::sdurwsim_bullet
add_library(RWSIM::sdurwsim_bullet SHARED IMPORTED)

set_target_properties(RWSIM::sdurwsim_bullet PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zw/CLionProjects/RobWork/RobWorkSim/src"
  INTERFACE_LINK_LIBRARIES "RWSIM::sdurwsim"
)

# Create imported target RWSIM::sdurwsim_test
add_library(RWSIM::sdurwsim_test SHARED IMPORTED)

set_target_properties(RWSIM::sdurwsim_test PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/zw/CLionProjects/RobWork/RobWorkSim/src"
  INTERFACE_LINK_LIBRARIES "RWSIM::sdurwsim"
)

# Import target "RWSIM::sdurwsim" for configuration "Debug"
set_property(TARGET RWSIM::sdurwsim APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RWSIM::sdurwsim PROPERTIES
  IMPORTED_LOCATION_DEBUG "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug/libsdurwsim.so"
  IMPORTED_SONAME_DEBUG "libsdurwsim.so"
  )

# Import target "RWSIM::sdurwsim_gui" for configuration "Debug"
set_property(TARGET RWSIM::sdurwsim_gui APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RWSIM::sdurwsim_gui PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "RWSIM::sdurwsim_ode"
  IMPORTED_LOCATION_DEBUG "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug/libsdurwsim_gui.so"
  IMPORTED_SONAME_DEBUG "libsdurwsim_gui.so"
  )

# Import target "RWSIM::sdurwsim_ode" for configuration "Debug"
set_property(TARGET RWSIM::sdurwsim_ode APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RWSIM::sdurwsim_ode PROPERTIES
  IMPORTED_LOCATION_DEBUG "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug/libsdurwsim_ode.so"
  IMPORTED_SONAME_DEBUG "libsdurwsim_ode.so"
  )

# Import target "RWSIM::sdurwsim_bullet" for configuration "Debug"
set_property(TARGET RWSIM::sdurwsim_bullet APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RWSIM::sdurwsim_bullet PROPERTIES
  IMPORTED_LOCATION_DEBUG "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug/libsdurwsim_bullet.so"
  IMPORTED_SONAME_DEBUG "libsdurwsim_bullet.so"
  )

# Import target "RWSIM::sdurwsim_test" for configuration "Debug"
set_property(TARGET RWSIM::sdurwsim_test APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RWSIM::sdurwsim_test PROPERTIES
  IMPORTED_LOCATION_DEBUG "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug/libsdurwsim_test.so"
  IMPORTED_SONAME_DEBUG "libsdurwsim_test.so"
  )

# Make sure the targets which have been exported in some other
# export set exist.
unset(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)
foreach(_target "RW::sdurw_algorithms" "RW::sdurw_assembly" "RW::sdurw_opengl" "RW::sdurw_proximitystrategies" "RW::sdurw_pathplanners" "RW::sdurw_simulation" "RW::sdurw_task" "RW::sdurw_core" "RW::sdurw_math" "RW::sdurw" "RWS::sdurws" )
  if(NOT TARGET "${_target}" )
    set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets "${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets} ${_target}")
  endif()
endforeach()

if(DEFINED ${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)
  if(CMAKE_FIND_PACKAGE_NAME)
    set( ${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
    set( ${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE "The following imported targets are referenced, but are missing: ${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets}")
  else()
    message(FATAL_ERROR "The following imported targets are referenced, but are missing: ${${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets}")
  endif()
endif()
unset(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE_targets)

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)