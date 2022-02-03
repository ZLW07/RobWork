# Test CMake version
cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

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

# The name of the project.
project(RobWorkStudio)

set(RWS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# ##################################################################################################
# lets start by finding RobWork since this is the primary dependency and because we want to use the
# macros available in RobWork if RW_ROOT is defined then we use that, else we try finding it the
# traditional way

# ##################################################################################################
# include the macros from robwork
include(${RWS_ROOT}/../RobWork/cmake/RobWorkMacros.cmake)
include(${RWS_ROOT}/cmake/RobWorkStudioMacros.cmake)

if(DEFINED VERSION)
    set(ROBWORKSTUDIO_VERSION ${VERSION})
    set(RWS_GOT_VERSION True)
else()
    rw_get_git_version(ROBWORKSTUDIO_VERSION ROBWORKSTUDIO_BRANCH)
    set(RWS_GOT_VERSION False)
endif()

if(NOT DEFINED RWP_ROOT)
    find_package(RobWork ${ROBWORKSTUDIO_VERSION} REQUIRED PATHS ${RWS_ROOT}/../RobWork/cmake )
else()
    string(TOLOWER ${RW_BUILD_TYPE} tmp)
    include(${RW_ROOT}cmake/RobWorkBuildConfig_${tmp}.cmake)
endif()

rw_init_project(${RWS_ROOT} RobWorkStudio RWS ${ROBWORKSTUDIO_VERSION})
rw_get_os_info()

if(DEFINED MSVC)
    set(RWS_SHARED_LIBS OFF)
endif()

rw_options(RWS)

# ##################################################################################################
# Add an "uninstall" target
configure_file("${RWS_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY
)
add_custom_target(uninstall_sdurws "${CMAKE_COMMAND}" -P
                                   "${CMAKE_BINARY_DIR}/uninstall_target.cmake"
)

# ##################################################################################################
# Add compile flags

if(DEFINED USE_WERROR)
    if(${USE_WERROR})
        set(WERROR_FLAG "-Werror")
    endif()
endif()

set(RWS_CXX_FLAGS
    "${WERROR_FLAG} ${RW_BUILD_WITH_CXX_FLAGS}"
    CACHE STRING "Change this to force using your own flags and not those of RobWorkStudio"
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWS_CXX_FLAGS}")

# ##################################################################################################
# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES RobWork dependencies are
# included in these vars
include(${RWS_ROOT}/cmake/RobWorkStudioSetup.cmake)
rw_set_install_dirs(RobWorkStudio RWS)

option(RWS_USE_STATIC_LINK_PLUGINS "Enable static linkage of built-in RWS plugins." ON)
if(RWS_SHARED_LIBS)
    set(RWS_USE_STATIC_LINK_PLUGINS OFF)
endif()

if(RWS_USE_STATIC_LINK_PLUGINS)
    message(STATUS "RobWorkStudio: Using static linking of default plugins!")
else()
    message(STATUS "RobWorkStudio: Using dynamic linking of default plugins!")
endif()

# Subdirectories adds plugins and libraries to these variables
set(RWS_PLUGIN_LIBRARIES)
set(RWS_COMPONENT_LIBRARIES)

# sub directories to process
add_subdirectory(ext)
add_subdirectory(src)

rw_sys_info(INFO)
message(STATUS "RobWorkStudio: platform ${INFO} ")

# ##################################################################################################
# setup testing
#

cmake_dependent_option(
    RWS_IS_TESTS_ENABLED "Set when you want to build the tests" ON "${RW_BUILD_WITH_GTEST}" OFF
)
if(RWS_IS_TESTS_ENABLED)
    message(STATUS "RobWorkStudio: tests ENABLED!")
    add_subdirectory(gtest)
else()
    message(STATUS "RobWorkStudio: tests DISABLED!")
endif()

# ##################################################################################################
# Try to find the current revision
#
rw_get_revision(${RWS_ROOT} ROBWORKSTUDIO)

# ##################################################################################################
# CONFIGURATION configure build/RobWorkStudioConfig.cmake.in
#

# first configure the header file
configure_file(${RWS_ROOT}/src/RobWorkStudioConfig.hpp.in "${RWS_ROOT}/src/RobWorkStudioConfig.hpp")

# next build information script
configure_file(${RWS_ROOT}/cmake/RobWorkStudioBuildConfig.cmake.in
               "${RWS_ROOT}/cmake/RobWorkStudioBuildConfig_${RWS_BUILD_TYPE}.cmake" @ONLY
)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use RobWorkStudio
configure_file(${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake.in
               "${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake" @ONLY
)

# and the version info
configure_file(${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake.in
               "${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake" @ONLY
)
configure_file(${RW_ROOT}/cmake/RobWorkConfigMacros.cmake.in
               "${RWS_ROOT}/cmake/RobWorkStudioConfigMacros.cmake" @ONLY
)

export(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE "${RWS_ROOT}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_PREFIX}::
)

# ##################################################################################################
# Installation stuff
#

# install configuration
install(
    FILES ${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake
          ${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake
    DESTINATION "${RWS_INSTALL_DIR}/cmake"
    COMPONENT cmake
)

install(
    FILES "cmake/RobWorkStudioBuildConfig_${RWS_BUILD_TYPE}.cmake" "cmake/RobWorkStudioMacros.cmake" "cmake/RobWorkStudioConfigMacros.cmake"
    DESTINATION "${RWS_INSTALL_DIR}/cmake"
    COMPONENT cmake
)

# misc
install(
    FILES LICENSE.txt NOTICE.txt
    DESTINATION "${RWS_INSTALL_DIR}"
    COMPONENT cmake
)
install(
    FILES ${RWS_ROOT}/src/RobWorkStudioConfig.hpp ${RWS_ROOT}/src/RobWorkStudio.hpp
          ${RWS_ROOT}/src/RobWorkStudioPlugin.hpp
    DESTINATION "${INCLUDE_INSTALL_DIR}"
    COMPONENT sdurws
)

install(
    EXPORT ${PROJECT_PREFIX}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_PREFIX}::
    DESTINATION ${RWS_INSTALL_DIR}/cmake
)

# Install dependencies on Windows
if(WIN32)
    set(_RWS_MODULES_ROOT "${RWS_ROOT}/cmake/Modules")
    install(
        FILES "${_RWS_MODULES_ROOT}/Find_QT5_DIR.cmake" "${_RWS_MODULES_ROOT}/FindQt5Core.cmake"
              "${_RWS_MODULES_ROOT}/FindQt5Gui.cmake" "${_RWS_MODULES_ROOT}/FindQt5OpenGL.cmake"
              "${_RWS_MODULES_ROOT}/FindQt5Widgets.cmake"
        DESTINATION "${RWS_INSTALL_DIR}/cmake/Modules"
        COMPONENT cmake
    )
    STRING(REPLACE ".lib" ".dll" PYTHON_DLL "${PYTHON_LIBRARIES}")
    if(EXISTS "${PYTHON_DLL}")
        install(FILES ${PYTHON_DLL} DESTINATION ${BIN_INSTALL_DIR} COMPONENT python_dll)
    else()
        get_filename_component(python_file ${PYTHON_DLL} NAME)
        get_filename_component(python_path ${PYTHON_DLL} DIRECTORY)
        install(FILES "${python_path}/../${python_file}" DESTINATION ${BIN_INSTALL_DIR} COMPONENT python_dll)
    endif()
endif()

# this will configure the dependency file that will be used if install is invoked
# GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION) # not allowed anymore - must
# use generator expressions instead
configure_file("${RWS_ROOT}/cmake/dependencies.cmake.in" "${RWS_ROOT}/cmake/dependencies.cmake.gen"
               @ONLY
)

# Install the example directories
install(
    DIRECTORY "${RWS_ROOT}/example/"
    DESTINATION "${RWS_INSTALL_DIR}/example/"
    PATTERN ".svn" EXCLUDE
    PATTERN "example/CMakeLists.txt" EXCLUDE
)

# Documentation build target
find_program(doc_executable doxygen)
if(doc_executable)
    add_custom_target(
        apidoc
        doxygen
        WORKING_DIRECTORY ${RW_ROOT}/..
    )

    if(EXISTS "${RW_ROOT}/../apidocs/html/index.qhp")
        add_custom_target(qtrwdoc qhelpgenerator ${RW_ROOT}/../apidocs/html/index.qhp -o
                                  ${RWS_ROOT}/docs/robwork-v${ROBWORKSTUDIO_VERSION}.qch
        )
    elseif(doc_executable)
        add_custom_target(qtrwdoc qhelpgenerator ${RW_ROOT}/../apidocs/html/index.qhp -o
                                  ${RWS_ROOT}/docs/robwork-v${ROBWORKSTUDIO_VERSION}.qch
        )
        add_dependencies(qtrwdoc apidoc)
    endif()

    if(TARGET qtrwdoc)
        configure_file(${RWS_ROOT}/docs/sdurw_help.qhcp.in
                       ${RWS_ROOT}/docs/sdurw_help-v${ROBWORK_VERSION}.qhcp
        )
        add_custom_target(sdurw_help
                          qcollectiongenerator ${RWS_ROOT}/docs/sdurw_help-v${ROBWORK_VERSION}.qhcp
                          -o ${RWS_ROOT}/docs/robwork_help-v${ROBWORK_VERSION}.qhc
        )
        add_dependencies(sdurw_help qtrwdoc)
    endif()
endif()

# Packaging include(cmake/packing.cmake)

# Create installer
rw_create_installer()
