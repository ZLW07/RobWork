# ------------------------------------------------------------------------------------
# Helper to use RobWork from outside project
#
# set ROBWORKSTUDIO_DIR to installation path to find root of sdurw, else automatic finding will be
# used based on RobWorkStudio_DIR
#
# ROBWORKSTUDIO_LIBRARIES is filled with all available RobWork libraries ROBWORKSTUDIO_INCLUDE_DIRS
# is filled with RobWorkStudio and available 3rdparty headers ROBWORKSTUDIO_LIBRARY_DIRS is filled
# with RobWorkStudio components libraries install directory and 3rdparty libraries paths
#
# www.robwork.dk
# ------------------------------------------------------------------------------------

# ---[ Find RobWorkStudio

if(ROBWORKSTUDIO_FIND_QUIETLY)
    set(QUIET_ QUIET)
else()
    set(QUIET_)
endif()

set(RWSCFG_ROOT ${CMAKE_CURRENT_LIST_DIR})
include("${RWSCFG_ROOT}/RobWorkStudioConfigMacros.cmake")
set(RobWorkStudio_REQUIRED ${RobWorkStudio_FIND_REQUIRED})

# get the relavant build type
get_robworkstudio_build_type(${RWSCFG_ROOT} RWS_BUILD_TYPE)

if(NOT TARGET sdurws)
    include("${RWSCFG_ROOT}/RobWorkStudioTargets.cmake")

    if(EXISTS "${RWSCFG_ROOT}/RobWorkStudioluaTargets.cmake")
        include("${RWSCFG_ROOT}/RobWorkStudioluaTargets.cmake")
    endif()
    if(EXISTS "${RWSCFG_ROOT}/RobWorkStudiojavaTargets.cmake")
        include("${RWSCFG_ROOT}/RobWorkStudiojavaTargets.cmake")
    endif()
    if(EXISTS "${RWSCFG_ROOT}/RobWorkStudiopythonTargets.cmake")
        include("${RWSCFG_ROOT}/RobWorkStudiopythonTargets.cmake")
    endif()
endif()

if(EXISTS ${RWSCFG_ROOT}/RobWorkStudioBuildConfig_${RWS_BUILD_TYPE}.cmake)
    include(${RWSCFG_ROOT}/RobWorkStudioBuildConfig_${RWS_BUILD_TYPE}.cmake)

    # check whether RobWorkConfig.cmake is found into a RobWork installation or in a build tree
    rws_setup_config_directories("${RWSCFG_ROOT}")

    include(${RWSCFG_ROOT}/RobWorkStudioMacros.cmake)
    cmake_minimum_required(VERSION 3.5.1)
    set(CMAKE_MODULE_PATH ${RWSCFG_ROOT}/Modules ${CMAKE_MODULE_PATH})

    # ##############################################################################################
    # Find Dependencies
    find_package(Qt6 COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets QUIET)
    if(Qt6Core_FOUND
       AND Qt6Gui_FOUND
       AND Qt6Widgets_FOUND
       AND Qt6OpenGL_FOUND
       AND Qt6OpenGLWidgets_FOUND
    )
        set(QT_LIBRARIES ${Qt6Core_LIBRARIES} ${Qt6Gui_LIBRARIES} ${Qt6Widgets_LIBRARIES}
                         ${Qt6OpenGL_LIBRARIES} ${Qt6OpenGLWidgets_LIBRARIES}
        )
    else()
        find_package(Qt5Core 5.5.1 REQUIRED)
        find_package(Qt5Gui 5.5.1 REQUIRED)
        find_package(Qt5Widgets 5.5.1 REQUIRED)
        find_package(Qt5OpenGL 5.5.1 REQUIRED)
        get_target_property(QT_UIC_EXECUTABLE Qt5::uic LOCATION)
        set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES}
                         ${Qt5OpenGL_LIBRARIES}
        )
        set(QT_INCLUDES ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
                        ${Qt5OpenGL_INCLUDE_DIRS}
        )
    endif()

    # Find and setup OpenGL.
    if(POLICY CMP0072) # Introduce cmake 3.11
        cmake_policy(SET CMP0072 NEW)
    endif()
    find_package(OpenGL REQUIRED)

    if(${RWS_BUILD_WITH_PYTHON})
        find_package(PythonLibs ${RWS_BUILD_WITH_PYTHONLIBS_MAJOR_VERSION} REQUIRED)
        find_package(PythonInterp ${RWS_BUILD_WITH_PYTHON_MAJOR_VERSION} REQUIRED)
    endif()

    if(${RWS_BUILD_WITH_FREEGLUT})
        find_package(FreeGLUT REQUIRED)
    endif()

    # Find RobWork
    if(NOT DEFINED RobWork_FOUND OR NOT ${RobWork_FOUND})
        if(${IS_INSTALL})
            find_package(RobWork QUIET)
        else()
            set(RobWork_FOUND FALSE)
        endif()

        if(NOT ${RobWork_FOUND})
            find_package(RobWork REQUIRED HINTS ${RWS_BUILD_WITH_RW_ROOT}/cmake)
        endif()
    else()
        message(STATUS "RobWork Already Found")
    endif()

    # ##############################################################################################
    # now RWS_ROOT and RWSCFG_ROOT is set. Lets extract the stuff needed to run a project

    # next get the build configuration of the requested built type

    # Find libraries
    set(ROBWORKSTUDIO_LIBRARIES_TMP)
    rws_find_components(ROBWORKSTUDIO_LIBRARIES_TMP)

    set(BOOST_ROOT ${RWS_BUILD_WITH_BOOST_ROOT})
    set(BOOST_INCLUDEDIR ${RWS_BUILD_WITH_BOOST_INCLUDE_DIR})
    set(BOOST_LIBRARYDIR ${RWS_BUILD_WITH_BOOST_LIBRARY_DIR})

    if(DEFINED WIN32 AND ${IS_INSTALL})
        set(BOOST_INCLUDEDIR "${RW_INCLUDE_EXT}/boost")
        set(BOOST_LIBRARYDIR "${RW_LIBS}")
    endif()

    # Set extra compiler flags. The user should be able to change this
    rws_setup_flags_and_definitions()

    set(ROBWORKSTUDIO_BUILD_PATH "${RWS_BUILD_WITH_RWS_ROOT}")
    set(ROBWORKSTUDIO_INCLUDE_DIRS_TMP "${PYTHON_INCLUDE_DIRS}" "${RWS_BUILD_WITH_INCLUDE_DIR}"
                                       "${ROBWORK_INCLUDE_DIR}"
    )
    set(ROBWORKSTUDIO_LIBRARY_DIRS_TMP "${RWS_BUILD_WITH_LIBRARY_DIRS}" "${ROBWORK_LIBRARY_DIRS}")
    set(ROBWORKSTUDIO_LIBRARIES "${ROBWORKSTUDIO_LIBRARIES_TMP}" "${ROBWORK_LIBRARIES}"
                                "${RWS_BUILD_WITH_LIB_DEPEND}" "${PYTHON_LIBRARIES}"
    )
    list(REMOVE_DUPLICATES ROBWORKSTUDIO_LIBRARIES)

    # make sure that the library and include paths are pointing to the right locations
    string(REPLACE "${ROBWORKSTUDIO_BUILD_PATH}/ext" "${RWS_INCLUDE_EXT}"
                   ROBWORKSTUDIO_INCLUDE_DIRS "${ROBWORKSTUDIO_INCLUDE_DIRS_TMP}"
    )
    string(REPLACE "${ROBWORKSTUDIO_BUILD_PATH}/src" "${RWS_INCLUDE_SRC}"
                   ROBWORKSTUDIO_INCLUDE_DIRS "${ROBWORKSTUDIO_INCLUDE_DIRS}"
    )
    if(WIN32)
        list(APPEND ROBWORKSTUDIO_INCLUDE_DIRS "${RWS_INCLUDE_EXT}")
    endif()

    list(REMOVE_DUPLICATES ROBWORKSTUDIO_INCLUDE_DIRS)

    string(REPLACE "${ROBWORKSTUDIO_BUILD_PATH}/libs/${RWS_BUILD_TYPE}" "${RWS_LIBS}"
                   ROBWORKSTUDIO_LIBRARY_DIRS "${RWS_BUILD_WITH_LIBRARY_DIRS}"
    )
    if(${IS_INSTALL} AND DEFINED WIN32)
        string(REPLACE "${RWS_BUILD_WITH_BOOST_LIBRARY_DIR}/" "" ROBWORKSTUDIO_LIBRARIES
                       "${ROBWORKSTUDIO_LIBRARIES}"
        )
    endif()
    list(REMOVE_DUPLICATES ROBWORKSTUDIO_LIBRARY_DIRS)

    if(RWS_SDURWS_CREATED)
        target_include_directories(RWS::sdurws INTERFACE "${ROBWORKSTUDIO_INCLUDE_DIRS}")
    endif()

    # Find and add full path information for the RobWorkStudio libraries
    set(ROBWORKSTUDIO_LIBRARIES_TMP ${ROBWORKSTUDIO_LIBRARIES})
    set(ROBWORKSTUDIO_LIBRARIES)
    foreach(l ${ROBWORKSTUDIO_LIBRARIES_TMP})
        unset(res)
        rws_verify_library(${l} res)
        if("${res}" STREQUAL "")
            list(APPEND ROBWORKSTUDIO_LIBRARIES "${l}")
        else()
            list(APPEND ROBWORKSTUDIO_LIBRARIES "${res}")
        endif()
    endforeach()
else()
    message(STATUS "RobWorkStudio: Could not find a suitable RobWorkStudio installation!")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    RobWorkStudio "Found RobWorkStudio-Version ${RobWorkStudio_VERSION}" RWS_ROOT
    ROBWORKSTUDIO_LIBRARIES ROBWORKSTUDIO_INCLUDE_DIRS ROBWORKSTUDIO_LIBRARY_DIRS
)
mark_as_advanced(ROBWORKSTUDIO_LIBRARIES ROBWORKSTUDIO_INCLUDE_DIRS ROBWORKSTUDIO_LIBRARY_DIRS)

if(ROBWORKSTUDIO_FOUND)
    set(ROBWORKSTUDIO_VERSION
        ${RobWorkStudio_VERSION}
        CACHE STRING "RobWorkStudio version"
    )
endif(ROBWORKSTUDIO_FOUND)
