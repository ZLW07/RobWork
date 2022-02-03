# ------------------------------------------------------------------------------------
# Helper to use RobWorkSim from outside project
#
# ROBWORKSIM_LIBRARIES is filled with all available RobWork libraries ROBWORKSIM_INCLUDE_DIRS is
# filled with RobWorkSim and available 3rdparty headers ROBWORKSIM_LIBRARY_DIRS is filled with
# RobWorkSim components libraries  and 3rdparty libraries paths
#
# www.robwork.dk
# ------------------------------------------------------------------------------------

# ---[ Find RobWorkSim

if(ROBWORKSIM_FIND_QUIETLY)
    set(QUIET_ QUIET)
else(ROBWORKSIM_FIND_QUIETLY)
    set(QUIET_)
endif(ROBWORKSIM_FIND_QUIETLY)

set(RWSIMCFG_ROOT ${CMAKE_CURRENT_LIST_DIR})
include("${RWSIMCFG_ROOT}/RobWorkSimConfigMacros.cmake")
set(RobWorkSim_REQUIRED ${RobWorkSim_FIND_REQUIRED})
# get the relavant build type
get_robworksim_build_type(${RWSIMCFG_ROOT} RWSIM_BUILD_TYPE)

if(NOT TARGET sdurwsim)
    include("${RWSIMCFG_ROOT}/RobWorkSimTargets.cmake")
    if(EXISTS "${RWSIMCFG_ROOT}/RobWorkSimluaTargets.cmake")
        include("${RWSIMCFG_ROOT}/RobWorkSimluaTargets.cmake")
    endif()
    if(EXISTS "${RWSIMCFG_ROOT}/RobWorkSimjavaTargets.cmake")
        include("${RWSIMCFG_ROOT}/RobWorkSimjavaTargets.cmake")
    endif()
    if(EXISTS "${RWSIMCFG_ROOT}/RobWorkSimpythonTargets.cmake")
        include("${RWSIMCFG_ROOT}/RobWorkSimpythonTargets.cmake")
    endif()
endif()

if(EXISTS ${RWSIMCFG_ROOT}/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake)
    include(${RWSIMCFG_ROOT}/RobWorkSimBuildConfig_${RWSIM_BUILD_TYPE}.cmake)

    # check whether RobWorkSimConfig.cmake is found into a RobWorkSim installation or in a build
    # tree
    rwsim_setup_config_directories("${RWSIMCFG_ROOT}")

    # ##############################################################################################
    # now RWSIM_ROOT and RWSIMCFG_ROOT is set. Lets extract the stuff needed to run a project

    # next get the build configuration of the requested built type

    # setup path to custom find scripts
    set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIMCFG_ROOT}/Modules")

    if(NOT DEFINED RobWork_DIR AND NOT ${IS_INSTALL})
        set(RobWork_DIR ${RW_ROOT}/cmake)
    endif()
    if(NOT DEFINED RobWorkStudio_DIR AND NOT ${IS_INSTALL})
        set(RobWorkStudio_DIR ${RWS_ROOT}/cmake)
    endif()

    # Find RobWork
    if(NOT DEFINED RobWork_FOUND OR NOT ${RobWork_FOUND})
        if(${IS_INSTALL})
            find_package(RobWork QUIET)
        else()
            set(RobWork_FOUND FALSE)
        endif()

        if(NOT ${RobWork_FOUND})
            find_package(RobWork REQUIRED HINTS ${RWSIM_BUILD_WITH_RW_ROOT}/cmake)
        endif()
    else()
        message(STATUS "RobWork Already Found")
    endif()

    # Find RobWorkStudio
    if(NOT DEFINED RobWorkStudio_FOUND OR NOT ${RobWorkStudio_FOUND})
        if(${IS_INSTALL})
            find_package(RobWorkStudio QUIET)
        else()
            set(RobWorkStudio_FOUND FALSE)
        endif()

        if(NOT ${RobWorkStudio_FOUND})
            find_package(RobWorkStudio REQUIRED HINTS ${RWSIM_BUILD_WITH_RWS_ROOT}/cmake)
        endif()
    else()
        message(STATUS "RobWorkStudio Already Found")
    endif()

    set(tmp_list ${RWSIM_BUILD_WITH_LIBRARIES} ${RobWorkSim_FIND_COMPONENTS})
    list(FIND tmp_list "RWSIM::sdurwsim_ode" FOUND)
    if("${FOUND}" EQUAL "-1")
        list(FIND tmp_list "sdurwsim_ode" FOUND)
    endif()
    if("${FOUND}" EQUAL "-1")
        list(FIND RobWorkSim_FIND_COMPONENTS "ode" FOUND)
    endif()
    if("${FOUND}" GREATER "-1")
        if(RWSIM_BUILD_WITH_ODE)
            set(ODE_USE_DOUBLE ${RWSIM_BUILD_WITH_ODE_USE_DOUBLE})
            set(ODE_USE_DEBUG ${RWSIM_BUILD_WITH_ODE_USE_DEBUG})
            if(NOT ODE_DIR)
                set(ODE_DIR ${RWSIM_BUILD_WITH_ODE_DIR})
            endif()
            if(NOT ODE_INCLUDE_DIR)
                set(ODE_INCLUDE_DIR ${RWSIM_BUILD_WITH_ODE_INCLUDE_DIR})
            endif()
            find_package(ODE REQUIRED)
            if(ODE_FOUND)
                message(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
            else()
                set(RobWorkSim_ode_FOUND FALSE)
                message(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
            endif()
        endif()
    endif()

    set(LIBRARIES_TO_INCLUDE)
    rwsim_FIND_COMPONENTS(LIBRARIES_TO_INCLUDE)

    set(ROBWORKSIM_BUILD_PATH "${RWSIM_BUILD_WITH_RWSIM_ROOT}")
    set(ROBWORKSIM_INCLUDE_DIRS_TMP "${RWSIM_BUILD_WITH_INCLUDE_DIRS}" "${ROBWORK_INCLUDE_DIR}")
    set(ROBWORKSIM_LIBRARY_DIRS_TMP "${RWSIM_BUILD_WITH_LIBRARY_DIRS}")
    set(ROBWORKSIM_LIBRARIES_TMP "${LIBRARIES_TO_INCLUDE}"
                                 "${RWSIM_BUILD_WITH_LIB_DEPEND}" "${ROBWORK_LIBRARIES}"
    )

    # make sure that the library and include paths are pointing to the right locations
    string(REPLACE "${ROBWORKSIM_BUILD_PATH}/ext" "${RWSIM_INCLUDE_EXT}" ROBWORKSIM_INCLUDE_DIRS
                   "${ROBWORKSIM_INCLUDE_DIRS_TMP}"
    )
    string(REPLACE "${ROBWORKSIM_BUILD_PATH}/src" "${RWSIM_INCLUDE_SRC}" ROBWORKSIM_INCLUDE_DIRS
                   "${ROBWORKSIM_INCLUDE_DIRS}"
    )
    list(REMOVE_DUPLICATES ROBWORKSIM_INCLUDE_DIRS)

    string(REPLACE "${ROBWORKSIM_BUILD_PATH}/libs/${RWSIM_BUILD_TYPE}" "${RWSIM_LIBS}"
                   ROBWORKSIM_LIBRARY_DIRS "${ROBWORKSIM_LIBRARY_DIRS_TMP}"
    )
    list(REMOVE_DUPLICATES ROBWORKSIM_LIBRARY_DIRS)

    string(REPLACE "${ROBWORKSIM_BUILD_PATH}/libs/${RWSIM_BUILD_TYPE}" "${RWSIM_LIBS}"
                   ROBWORKSIM_LIBRARIES "${ROBWORKSIM_LIBRARIES_TMP}"
    )
    list(REMOVE_DUPLICATES ROBWORKSIM_LIBRARIES)

    set(ROBWORKSIM_LIBRARIES_TMP ${ROBWORKSIM_LIBRARIES})
    set(ROBWORKSIM_LIBRARIES)

    foreach(l ${ROBWORKSIM_LIBRARIES_TMP})
        unset(res)
        rwsim_verify_library(${l} res)
        if("${res}" STREQUAL "")
            list(APPEND ROBWORKSIM_LIBRARIES "${l}")
        else()
            list(APPEND ROBWORKSIM_LIBRARIES "${res}")
        endif()
    endforeach()

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(
        RobWorkSim DEFAULT_MSG RWSIM_ROOT ROBWORKSIM_LIBRARIES ROBWORKSIM_INCLUDE_DIRS
        ROBWORKSIM_LIBRARY_DIRS
    )
    mark_as_advanced(ROBWORKSIM_LIBRARIES ROBWORKSIM_INCLUDE_DIRS ROBWORKSIM_LIBRARY_DIRS)

else()
    set(ROBWORKSIM_FOUND NOTFOUND)
    message(
        STATUS
            "This build of RobWorkSim is not compiled in ${RWSIM_BUILD_TYPE} please specify another buildtype!"
    )
endif()

if(ROBWORKSIM_FOUND)
    set(ROBWORKSIM_VERSION
        ${RobWorkSim_FOUND_VERSION}
        CACHE STRING "RobWorkSim version"
    )
endif(ROBWORKSIM_FOUND)
