# Locate ode This module defines ODE_LIBRARY ODE_FOUND, if false, do not try to link to ode
# ODE_INCLUDE_DIR, where to find the headers ODE_BUILD_WITH, set to DOUBLE or SINGLE
#
# this module optionally use followin vars to guide the search for ODE: ODE_DIR - path to ODE root
# dir ODE_LIBRARY_DIR - specify to guide the search in library ODE_LIBRARY_NAME - specify to guide
# library search ad selection ODE_USE_SINGLE - set if force using double ODE_USE_DOUBLE - set if
# force using double ODE_USE_DEBUG - set if force using debug created by RobWork, based on code by
# David Guthrie.  Based on code by Robert Osfield

find_package(ODE QUIET PATHS "$ENV(ODE_ROOT)/lib/cmake/ode-*" NO_MODULE)

macro(FIND_ODE_LIBRARY MYLIBRARY MYLIBRARYNAME)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a .lib .dylib)
    find_library(
        ${MYLIBRARY}
        NAMES ${ODE_LIBRARY_NAME} ${MYLIBRARYNAME}
        HINTS
            ${ODE_LIBRARY_DIR}
            ${ODE_DIR}/lib
            $ENV{ODE_DIR}/lib
            $ENV{ODE_DIR}
            ${DELTA3D_EXT_DIR}/lib
            $ENV{DELTA_ROOT}/ext/lib
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local/lib
            /usr/local/opt/ode/lib
            /usr/lib
            /sw/lib
            /opt/local/lib
            /opt/csw/lib
            /opt/lib
            [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
            /usr/freeware/lib64
        PATH_SUFFIXES lib/ReleaseSingleLib
                      lib/ReleaseSingleDLL
                      lib/ReleaseDoubleLib
                      lib/ReleaseDoubleDLL
                      lib/DebugSingleLib
                      lib/DebugSingleDLL
                      lib/DebugDoubleLib
                      lib/DebugDoubleDLL
                      lib/Release
                      lib/Debug
                      ReleaseSingleLib
                      ReleaseSingleDLL
                      ReleaseDoubleLib
                      ReleaseDoubleDLL
                      DebugSingleLib
                      DebugSingleDLL
                      DebugDoubleLib
                      DebugDoubleDLL
                      Release
                      Debug
    )
endmacro()

if(NOT ${ODE_FOUND})
    find_path(
        ODE_INCLUDE_DIR ode/ode.h
        HINTS
            ${ODE_DIR}/include
            $ENV{ODE_DIR}/include
            $ENV{ODE_DIR}
            ${DELTA3D_EXT_DIR}/inc
            $ENV{DELTA_ROOT}/ext/inc
            ~/Library/Frameworks
            /Library/Frameworks
            /sr/local/ode/include
            /usr/local/include
            /usr/include
            /usr/include/cal3d
            /sw/include # Fink
            /opt/local/include # DarwinPorts
            /opt/csw/include # Blastwave
            /opt/include
            [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
            /usr/freeware/include
    )
    unset(ODE_BUILD_WITH)
    unset(ODE_LIBRARY)

    if(ODE_USE_DEBUG)
        if(ODE_USE_SINGLE)
            set(DEBUG_LIST ode_singled oded)
            find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            set(ODE_BUILD_WITH "SINGLE")
        elseif(ODE_USE_DOUBLE)
            set(DEBUG_LIST ode_doubled oded)
            find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            set(ODE_BUILD_WITH "DOUBLE")
        else()
            # else try first with single then with double
            set(DEBUG_LIST ode_singled oded)
            find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            set(ODE_BUILD_WITH "SINGLE")
            if(NOT ODE_LIBARY_TMP)
                set(DEBUG_LIST ode_doubled oded)
                find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
                set(ODE_BUILD_WITH "DOUBLE")
            endif()
        endif()

    else()
        if(ODE_USE_SINGLE)
            set(RELEASE_LIST ode_single ode)
            set(DEBUG_LIST ode_singled oded)
            find_ode_library(ODE_LIBRARY_TMP "${RELEASE_LIST};${DEBUG_LIST}")
            set(ODE_BUILD_WITH "SINGLE")
        elseif(ODE_USE_DOUBLE)
            set(RELEASE_LIST ode_double ode)
            set(DEBUG_LIST ode_doubled oded)
            find_ode_library(ODE_LIBRARY_TMP "${RELEASE_LIST};${DEBUG_LIST}")
            set(ODE_BUILD_WITH "DOUBLE")

        else()
            # first try single
            set(RELEASE_LIST ode_single ode ode_singled oded)
            find_ode_library(ODE_LIBRARY_TMP "${RELEASE_LIST}")
            set(ODE_BUILD_WITH "SINGLE")
            if(NOT ODE_LIBRARY_TMP)
                set(RELEASE_LIST ode_double ode)
                find_ode_library(ODE_LIBRARY_TMP "${RELEASE_LIST}")
                set(ODE_BUILD_WITH "DOUBLE")
            endif()
            # try debug
            if(NOT ODE_LIBRARY_TMP)
                set(DEBUG_LIST ode_singled oded)
                find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
                set(ODE_BUILD_WITH "SINGLE")
            endif()
            if(NOT ODE_LIBRARY_TMP)
                set(DEBUG_LIST ode_doubled oded)
                find_ode_library(ODE_LIBRARY_TMP "${DEBUG_LIST}")
                set(ODE_BUILD_WITH "DOUBLE")
            endif()
        endif()
    endif()
    set(ODE_FOUND false)
    if(ODE_LIBRARY_TMP AND ODE_INCLUDE_DIR)
        set(ODE_FOUND true)
        set(ODE_LIBRARY ${ODE_LIBRARY_TMP})
        set(ODE_LIBRARIES ${ODE_LIBRARY})
    endif(ODE_LIBRARY_TMP AND ODE_INCLUDE_DIR)
else()
    if("${ODE_INCLUDE_DIR}" STREQUAL "")
        set(ODE_INCLUDE_DIR ${ODE_INCLUDE_DIRS})
    endif()
endif()
