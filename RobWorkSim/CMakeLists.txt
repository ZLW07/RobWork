# ##################################################################################################
# Test CMake version and CMAKE_BUILD_TYPE
cmake_minimum_required(VERSION 3.10  )


if(POLICY CMP0071) # Introduce cmake 3.10
    cmake_policy(SET CMP0071 NEW)
endif()
if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 NEW)
endif()
if(POLICY CMP0086) # Introduce cmake 3.14
    cmake_policy(SET CMP0086 NEW)
endif()

set(CMAKE_CXX_STANDARD 11)

# ##################################################################################################
# The name of the project.
project(RobWorkSim)

set(RWSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# ##################################################################################################
# include the macros from robwork
include(${RWSIM_ROOT}/../RobWork/cmake/RobWorkMacros.cmake)

if(DEFINED VERSION)
    set(ROBWORKSIM_VERSION ${VERSION})
    set(RWSIM_GOT_VERSION True)
else()
    rw_get_git_version(ROBWORKSIM_VERSION ROBWORKSIM_BRANCH)
    set(RWSIM_GOT_VERSION False)
endif()

if(NOT DEFINED RWP_ROOT)
    find_package(RobWork ${ROBWORKSIM_VERSION} REQUIRED PATHS ${RWSIM_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
else()
    string(TOLOWER ${RW_BUILD_TYPE} tmp)
    include(${RW_ROOT}cmake/RobWorkBuildConfig_${tmp}.cmake)
endif()

# for backward compatibility
set(ROOT RWSIM_ROOT)
rw_init_project(${RWSIM_ROOT} RobWorkSim RWSIM ${ROBWORKSIM_VERSION})
rw_get_os_info()
rw_options(RWSIM)

# ##################################################################################################
# Add an "uninstall" target configure_file("${RWSIM_ROOT}/cmake/uninstall_target.cmake.in"
# "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY) add_custom_target(uninstall
# "${CMAKE_COMMAND}" -P "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

# ##################################################################################################
# setup stuff for RobWorkStudio

# add the RWSIM modules to the module path
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIM_ROOT}/cmake/Modules/")

# This sets up ROBWORKSIM_INCLUDE_DIR and ROBWORKSIM_LIBRARIES RobWork dependencies are included in
# these vars
include(${RWSIM_ROOT}/cmake/RobWorkSimSetup.cmake)
rw_set_install_dirs(RobWorkSim RWSIM)

# now lets find RobWorkStudio
if(NOT DEFINED RWP_ROOT)
    find_package(RobWorkStudio ${ROBWORKSIM_VERSION} REQUIRED PATHS ${RWSIM_ROOT}/../RobWorkStudio/cmake NO_DEFAULT_PATH)
endif()
# ##################################################################################################
# CONFIGURATION configure build/RobWorkStudioConfig.cmake.in
#

# first configure the header file
configure_file(${RWSIM_ROOT}/src/RobWorkSimConfig.hpp.in "${RWSIM_ROOT}/src/RobWorkSimConfig.hpp"
               @ONLY
)

# next build information script
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake" @ONLY
)

# Configure build/RobWorkSimConfig.cmake.in such that other projects might use robworkstudio
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake" @ONLY
)

# and the version info
configure_file(${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake" @ONLY
)

configure_file(${RW_ROOT}/cmake/RobWorkConfigMacros.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimConfigMacros.cmake" @ONLY
)

# Store the test results in a RWSimConfig.hpp file.
configure_file(${RWSIM_ROOT}/src/RobWorkSimConfig.hpp.in ${RWSIM_ROOT}/src/RobWorkSimConfig.hpp)

configure_file(${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp.in
               ${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp
)

# process sub directories add_subdirectory(ext)
add_subdirectory(src)

export(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE "${RWSIM_ROOT}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_PREFIX}::
)

# ##################################################################################################
# Installation stuff
#

# install configuration
install(
    FILES "${RWSIM_ROOT}/cmake/RobWorkSimConfigVersion.cmake"
          "${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake"
          "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake"
          "${RWSIM_ROOT}/cmake/RobWorkSimConfigMacros.cmake"
    DESTINATION "${RWSIM_INSTALL_DIR}/cmake"
)

install(FILES "${RWSIM_ROOT}/cmake/Modules/FindODE.cmake"
        DESTINATION "${RWSIM_INSTALL_DIR}/cmake/Modules"
)

install(FILES DESTINATION ${RWSIM_INSTALL_DIR}/cmake)

# misc
install(FILES LICENSE.txt NOTICE.txt DESTINATION "${RWSIM_INSTALL_DIR}/")

install(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_PREFIX}::
    DESTINATION ${RWSIM_INSTALL_DIR}/cmake
)

# ##################################################################################################
# setup testing
#

include(CMakeDependentOption)
cmake_dependent_option(
    RWSIM_IS_TESTS_ENABLED "Set when you want to build the tests" ON "${RW_BUILD_WITH_GTEST}" OFF
)
if(RWSIM_IS_TESTS_ENABLED)
    message(STATUS "RobWork tests ENABLED!")
    add_subdirectory(gtest)

else()
    message(STATUS "RobWork tests DISABLED!")
endif()

configure_file(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake.in
               "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake"
)

# Packaging
rw_create_installer()
