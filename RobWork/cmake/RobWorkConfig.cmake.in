# ------------------------------------------------------------------------------------
# Helper to use RobWork from outside project
#
# set ROBWORK_DIR to installation path to find root of sdurw, else automatic finding will be used
# based on RobWork_DIR
#
# ROBWORK_LIBRARIES is filled with all available RobWork libraries ROBWORK_INCLUDE_DIRS is filled
# with RobWork and available 3rdparty headers ROBWORK_LIBRARY_DIRS is filled with RobWork components
# libraries install directory and 3rdparty libraries paths
#
# www.robwork.dk
# ------------------------------------------------------------------------------------

# ---[ Find RobWork

if(ROBWORK_FIND_QUIETLY)
    set(QUIET_ QUIET)
else(ROBWORK_FIND_QUIETLY)
    set(QUIET_)
endif(ROBWORK_FIND_QUIETLY)

# ##################################################################################################
set(RWCFG_ROOT ${CMAKE_CURRENT_LIST_DIR})
include("${RWCFG_ROOT}/RobWorkConfigMacros.cmake")
rw_is_windows_install_by_exe(${RWCFG_ROOT} IS_WINDOWS_EXE_INSTALL)
set(RobWork_REQUIRED ${RobWork_FIND_REQUIRED})

# get the relavant build type
get_robwork_build_type(${RWCFG_ROOT} RW_BUILD_TYPE)

# Import target files
if(NOT TARGET sdurw)
    include("${RWCFG_ROOT}/RobWorkTargets.cmake")

    if(EXISTS "${RWCFG_ROOT}/RobWorkluaTargets.cmake")
        include("${RWCFG_ROOT}/RobWorkluaTargets.cmake")
    endif()

    if(EXISTS "${RWCFG_ROOT}/RobWorkjavaTargets.cmake")
        include("${RWCFG_ROOT}/RobWorkjavaTargets.cmake")
    endif()

    if(EXISTS "${RWCFG_ROOT}/RobWorkpythonTargets.cmake")
        include("${RWCFG_ROOT}/RobWorkpythonTargets.cmake")
    endif()
endif()

if(EXISTS ${RWCFG_ROOT}/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake)
    include(${RWCFG_ROOT}/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake)

    rw_setup_config_directories("${RWCFG_ROOT}")

    set(Boost_USE_STATIC_LIBS ${RW_BUILD_WITH_BOOST_USE_STATIC_LIB})
    unset(Boost_FIND_QUIETLY)
    if(${RW_BUILD_TYPE} STREQUAL "release")
        set(Boost_USE_DEBUG_LIBS OFF) # ignore debug libs and
        set(Boost_USE_RELEASE_LIBS ON) # only find release libs
    endif()

    set(BOOST_ROOT ${RW_BUILD_WITH_BOOST_ROOT})
    find_package(
        Boost QUIET
        COMPONENTS filesystem serialization system thread program_options date_time
        CONFIG
        HINTS "C:\\local" ${RW_BUILD_WITH_BOOST_ROOT}
    )
    if(${IS_WINDOWS_EXE_INSTALL})
        generate_boost_exe_targets()
    endif()
    if(NOT Boost_FOUND)
        find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread
                                               program_options
        )
    endif()
    if(TARGET Boost::headers)
        set(RW_BUILD_WITH_LIB_DEPEND ${RW_BUILD_WITH_LIB_DEPEND} Boost::headers)
        set(Boost_INCLUDE_DIR ${RW_BUILD_WITH_BOOST_INCLUDE_DIR})
    endif()

    set(CMAKE_MODULE_PATH ${RWCFG_ROOT}/Modules ${CMAKE_MODULE_PATH})
    set(QHULL_ROOT ${RW_BUILD_WITH_QHULL_ROOT})
    find_package(Qhull REQUIRED MODULE)

    # ##############################################################################################
    # now RW_ROOT and RWCFG_ROOT is set. Lets extract the stuff needed to run a project

    # next get the build configuration of the requested built type

    # check which components to include
    set(LIBRARIES_TO_INCLUDE) # Libraries that must be included
    rw_find_components(LIBRARIES_TO_INCLUDE)

    if(${RW_BUILD_WITH_FREEGLUT} AND NOT ${IS_WINDOWS_EXE_INSTALL})
        find_package(FreeGLUT REQUIRED)
    endif()

    if(DEFINED WIN32 AND ${IS_INSTALL})
        set(BOOST_INCLUDEDIR "${RW_INCLUDE_EXT}/boost")
        set(BOOST_LIBRARYDIR "${RW_LIBS}")
    endif()

    # Set extra compiler flags. The user should be able to change this
    rw_setup_flags_and_definitions()

    # ################ FIND ROBWORK #######################
    set(ROBWORK_BUILD_PATH "${RW_BUILD_WITH_RW_ROOT}")
    set(ROBWORK_LIBRARIES_TMP
        "${RW_BUILD_WITH_LIBRARIES_OPENGL};${LIBRARIES_TO_INCLUDE};${OPTIONAL_LIBRARIES_TO_INCLUDE};${RW_BUILD_WITH_LIB_DEPEND}"
    )
    if(${RW_BUILD_WITH_LUA})
        set(IS_FOUND TRUE)
        foreach(lib ${RW_BUILD_WITH_LIBRARIES_LUA})
            if(NOT EXISTS "${lib}")
                set(IS_FOUND FALSE)
            endif()
        endforeach()
        if(IS_FOUND)
            set(ROBWORK_LIBRARIES_TMP ${RW_BUILD_WITH_LIBRARIES_LUA} ${ROBWORK_LIBRARIES_TMP})
            set(RW_BUILD_WITH_INCLUDE_DIR ${RW_BUILD_WITH_LUA_INCLUDE_DIR}
                                          ${RW_BUILD_WITH_INCLUDE_DIR}
            )
        endif()
    endif()

    if(${RW_BUILD_WITH_XERCES} AND EXISTS ${RW_BUILD_WITH_LIBRARIES_XERCESC})
        set(ROBWORK_LIBRARIES_TMP ${RW_BUILD_WITH_LIBRARIES_XERCESC} ${ROBWORK_LIBRARIES_TMP})
        set(RW_BUILD_WITH_INCLUDE_DIR ${RW_BUILD_WITH_XERCES_INCLUDE_DIR}
                                      ${RW_BUILD_WITH_INCLUDE_DIR}
        )
    endif()

    # make sure that the library and include paths are pointing to the right locations

    # ######### Include Directories #############
    string(REPLACE "${ROBWORK_BUILD_PATH}/ext" "${RW_INCLUDE_EXT}" ROBWORK_INCLUDE_DIRS
                   "${RW_BUILD_WITH_INCLUDE_DIR}"
    )
    string(REPLACE "${ROBWORK_BUILD_PATH}/src" "${RW_INCLUDE_SRC}" ROBWORK_INCLUDE_DIRS
                   "${ROBWORK_INCLUDE_DIRS}"
    )
    if(WIN32)
        list(APPEND ROBWORK_INCLUDE_DIRS "${RW_INCLUDE_EXT}")
    endif()

    list(REMOVE_DUPLICATES ROBWORK_INCLUDE_DIRS)

    # ######### Library Paths #############
    string(REPLACE "${ROBWORK_BUILD_PATH}/libs/${RW_BUILD_TYPE}" "${RW_LIBS}" ROBWORK_LIBRARY_DIRS
                   "${RW_BUILD_WITH_LIBRARY_DIRS}"
    )
    list(REMOVE_DUPLICATES ROBWORK_LIBRARY_DIRS)

    string(REPLACE "${ROBWORK_BUILD_PATH}/libs/${RW_BUILD_TYPE}" "${RW_LIBS}" ROBWORK_LIBRARIES
                   "${ROBWORK_LIBRARIES_TMP}"
    )
    if(${IS_INSTALL} AND DEFINED WIN32)
        string(REPLACE "${RW_BUILD_WITH_BOOST_LIBRARY_DIR}/" "" ROBWORK_LIBRARIES
                       "${ROBWORK_LIBRARIES}"
        )
    endif()
    list(REMOVE_DUPLICATES ROBWORK_LIBRARIES)

    if(RW_SDURW_CREATED)
        target_include_directories(RW::sdurw INTERFACE "${ROBWORK_INCLUDE_DIRS}")
    endif()

    # Find and add full path information for the RobWork libraries
    set(ROBWORK_LIBRARIES_TMP ${ROBWORK_LIBRARIES})
    set(ROBWORK_LIBRARIES)
    foreach(l ${ROBWORK_LIBRARIES_TMP})
        unset(res)
        rw_verify_library(${l} res)
        if("${res}" STREQUAL "")
            list(APPEND ROBWORK_LIBRARIES "${l}")
        else()
            list(APPEND ROBWORK_LIBRARIES "${res}")
        endif()
    endforeach()

    set(ROBWORK_INCLUDE_DIR "${ROBWORK_INCLUDE_DIRS}")

else()
    message(
        STATUS
            "This build of RobWork is not compiled in ${RW_BUILD_TYPE} please specify another buildtype!"
    )
endif()

if(NOT TARGET gtest)
    if(EXISTS "${RWCFG_ROOT}/gtestTargets.cmake")
        find_package(Threads QUIET)
        include("${RWCFG_ROOT}/gtestTargets.cmake")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    RobWork
    "Found RobWork-Version ${RobWork_VERSION}"
    RW_ROOT
    ROBWORK_LIBRARIES
    ROBWORK_INCLUDE_DIRS
    ROBWORK_INCLUDE_DIR
    ROBWORK_LIBRARY_DIRS
)
mark_as_advanced(ROBWORK_LIBRARIES ROBWORK_INCLUDE_DIRS ROBWORK_INCLUDE_DIR ROBWORK_LIBRARY_DIRS)

if(ROBWORK_FOUND)
    set(ROBWORK_VERSION
        ${RobWork_VERSION}
        CACHE STRING "RobWork version"
    )
endif(ROBWORK_FOUND)
