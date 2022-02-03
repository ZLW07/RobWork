# Test CMake version
cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 NEW)
endif()
if(POLICY CMP0086) # Introduce cmake 3.14
    cmake_policy(SET CMP0086 NEW)
endif()

if(NOT DEFINED RWP_ROOT)
include("cmake/HunterGate.cmake")
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.297.tar.gz"
    SHA1 "3319fe6a3b08090df7df98dee75134d68e2ef5a3"
)
else()
    function(hunter_add_package)     
    endfunction()
endif()

# The name of the project.
project(RobWork)
set(CMAKE_CXX_STANDARD 11)

# set robwork root
set(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
include(${RW_ROOT}/cmake/RobWorkMacros.cmake)

# Setup RobWork Version
if(DEFINED VERSION)
    set(ROBWORK_VERSION ${VERSION})
    set(RW_GOT_VERSION True)
else()
    rw_get_git_version(ROBWORK_VERSION ROBWORK_BRANCH)
    set(RW_GOT_VERSION False)
endif()

rw_init_project(${RW_ROOT} RobWork RW ${ROBWORK_VERSION})
rw_get_os_info()

rw_options(RW)

# Add an "uninstall" target
configure_file("${RW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY
)
add_custom_target(uninstall_sdurw "${CMAKE_COMMAND}" -P
                                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake"
)

# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
include(${RW_ROOT}/cmake/RobWorkSetup.cmake)

# Setup install Directories
rw_set_install_dirs(RobWork RW) # Must be called after setup
set(RW_EXT_INSTALL_DIR ${INCLUDE_INSTALL_DIR}/ext/)

# Subdirectories to process:
add_subdirectory(ext)
add_subdirectory(src)

rw_sys_info(INFO)
message(STATUS "RobWork: system id=${INFO} ")

# Compile the RobWork tools folder
#add_subdirectory(tools)

# ##################################################################################################
# setup testing
#

include(CMakeDependentOption)
cmake_dependent_option(
    RW_IS_TESTS_ENABLED "Set when you want to build the tests" ON "${RW_BUILD_TESTS}" OFF
)
if(RW_IS_TESTS_ENABLED)
    message(STATUS "RobWork tests ENABLED!")
    if(RW_HAVE_GTEST)
        add_subdirectory(gtest)
    endif()
else()
    message(STATUS "RobWork tests DISABLED!")
endif()

# ##################################################################################################
# Test if we should compile the EXAMPLES
#
cmake_dependent_option(
    RW_IS_EXAMPLES_ENABLED "Set when you want to build the examples" ON "${RW_BUILD_EXAMPLES}" OFF
)
if(RW_IS_EXAMPLES_ENABLED)
    message(STATUS "RobWork examples ENABLED!")
    add_subdirectory(example)
else()
    message(STATUS "RobWork example DISABLED!")
endif()

# Try to find the current revision
rw_get_revision(${RW_ROOT} ROBWORK)

# Store the test results in a RobWorkConfig.hpp file.
configure_file(${RW_ROOT}/src/RobWorkConfig.hpp.in ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure cmake/RobWorkBuildConfig.cmake.in such that other projects might use RobWork

configure_file(${RW_ROOT}/cmake/RobWorkBuildConfig.cmake.in
               "${RW_ROOT}/cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake" @ONLY
)

# configure version information
configure_file(${RW_ROOT}/cmake/RobWorkConfigVersion.cmake.in
               "${RW_ROOT}/cmake/RobWorkConfigVersion.cmake" @ONLY
)

configure_file(${RW_ROOT}/cmake/RobWorkConfig.cmake.in "${RW_ROOT}/cmake/RobWorkConfig.cmake" @ONLY)

configure_file(${RW_ROOT}/cmake/RobWorkConfigMacros.cmake.in
               "${RW_ROOT}/cmake/RobWorkConfigMacros.cmake" @ONLY
)

export(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE "${RW_ROOT}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_PREFIX}::
)

if(RW_HAVE_GTEST)
    if (NOT RW_GTEST_FROM_HUNTER AND NOT (debug IN_LIST GTEST_LIBRARIES))

        if(TARGET ${GTEST_LIBRARIES} AND TARGET ${GTEST_MAIN_LIBRARIES})
            export(
                TARGETS ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
                NAMESPACE ${PROJECT_PREFIX}:: APPEND
                FILE "${RW_ROOT}/cmake/gtestTargets.cmake"
            )
        endif()
    else()
        STRING(TOUPPER ${RW_BUILD_TYPE} CAPITAL_BUILD_TYPE)
        file(WRITE "${RW_ROOT}/cmake/gtestTargets.cmake" 
        "add_library(gtest INTERFACE)
        add_library(RW::gtest ALIAS gtest)
        set_target_properties(gtest PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${GTEST_NATIVE_ROOT}/include
            INTERFACE_LINK_LIBRARIES ${GTEST_LIBRARY}
        )

        add_library(gtest_main INTERFACE)
        add_library(RW::gtest_main ALIAS gtest)
        set_target_properties(gtest_main PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${GTEST_NATIVE_ROOT}/include
            INTERFACE_LINK_LIBRARIES ${GTEST_MAIN_LIBRARY}
        )
        ")
    endif()
endif()

# ##################################################################################################
# Installation stuff
#

# configuration
install(
    FILES 
        "cmake/RobWorkConfigVersion.cmake" 
        "cmake/RobWorkConfig.cmake"
        "cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake"
        "cmake/RobWorkMacros.cmake"
        "cmake/RobWorkConfigMacros.cmake"
    DESTINATION ${RW_INSTALL_DIR}/cmake
    COMPONENT cmake
)

install(
    DIRECTORY ${RW_ROOT}/cmake/Modules
    DESTINATION ${RW_INSTALL_DIR}/cmake
    COMPONENT cmake
)

# Install stuff
install(
    FILES LICENSE.txt NOTICE.txt
    DESTINATION "${RW_INSTALL_DIR}/"
    COMPONENT cmake
)

install(
    FILES "${RW_ROOT}/src/RobWorkConfig.hpp"
    DESTINATION "${INCLUDE_INSTALL_DIR}"
    COMPONENT sdurw
)

install(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_PREFIX}::
    DESTINATION ${RW_INSTALL_DIR}/cmake
)


# Install dependencies on Windows
if(WIN32)

    getBoostLibraryList(BOOST_LIBRARIES_INSTALL "${Boost_LIBRARIES}")

    # Install external libraries
    install(
        FILES ${BOOST_LIBRARIES_INSTALL}
        DESTINATION ${LIB_INSTALL_DIR}
        COMPONENT boost
    )
    install(
        DIRECTORY "${Boost_INCLUDE_DIR}/boost/"
        DESTINATION "${INCLUDE_INSTALL_DIR}/ext/boost/boost"
        COMPONENT boost
    )

    if(RW_HAVE_XERCES)
        install(
            FILES ${XERCESC_LIBRARIES}
            DESTINATION ${LIB_INSTALL_DIR}
            COMPONENT external
        )
        install(
            DIRECTORY "${XERCESC_INCLUDE_DIR}/"
            DESTINATION "${INCLUDE_INSTALL_DIR}/ext/xerces/"
            COMPONENT xerces
        )
    endif()

    install(
        DIRECTORY "${EIGEN3_INCLUDE_DIR}/"
        DESTINATION "${INCLUDE_INSTALL_DIR}/ext/Eigen/"
        COMPONENT eigen
    )
endif()

# And if possible we would also like to install all the dll's and stuff that we use this will
# configure the dependency file that will be used if install is invoked
# GET_TARGET_PROPERTY(rw_executable_name loader-test LOCATION) # not allowed anymore - must use
# generator expressions instead
configure_file("${RW_ROOT}/cmake/dependencies.cmake.in" "${RW_ROOT}/cmake/dependencies.cmake.gen"
               @ONLY
)

# Documentation build target
find_program(doc_executable doxygen)
if(doc_executable)
    add_custom_target(
        doc
        doxygen
        WORKING_DIRECTORY ${RW_ROOT}
    )
endif()

# Packaging
rw_create_installer()
