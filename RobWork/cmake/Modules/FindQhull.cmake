# ##################################################################################################
# Find QHULL
#
# This sets the following variables: QHULL_FOUND - True if QHULL was found. QHULL_INCLUDE_DIRS -
# Directories containing the QHULL include files. QHULL_LIBRARIES - Libraries needed to use QHULL.
# QHULL_DEFINITIONS - Compiler flags for QHULL. If QHULL_USE_STATIC is specified then look for
# static libraries ONLY else look for shared ones
 
if(WIN32)
    find_package(Qhull QUIET NO_MODULE HINTS ${QHULL_ROOT})
else()
    # we are not ready for qhull config approach find_package(Qhull QUIET NO_MODULE)
    set(Qhull_FOUND FALSE)
endif()

if(NOT ${Qhull_FOUND})
    if(WIN32)
        set(qhull_libnames qhullstatic_r)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(qhull_libnames qhull_r qhull6_r)
    else()
        set(qhull_libnames libqhull_r.so qhull6_r)
    endif()

    # ##############################################################################################
    # Find Include Dir
    # ##############################################################################################
    find_file(
        QHULL_HEADER
        NAMES libqhull_r.h
        HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}" "${QHULL_NATIVE_ROOT}"
        PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
        PATH_SUFFIXES qhull src/libqhull libqhull_r include
    )
    if(QHULL_HEADER)
        get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
        get_filename_component(TMP ${QHULL_INCLUDE_DIR} PATH)
        set(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} ${TMP})
    else(QHULL_HEADER)
        set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
    endif(QHULL_HEADER)

    # ##############################################################################################
    # Find Library
    # ##############################################################################################
    find_library(
        QHULL_LIBRARY
        NAMES ${qhull_libnames}
        HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_NATIVE_ROOT}"
        PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull" "/usr"
        PATH_SUFFIXES project build bin lib lib/x86_64-linux-gnu lib64
    )
    
    # ##############################################################################################
    # Finalize find package
    # ##############################################################################################

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIR)

    mark_as_advanced(QHULL_LIBRARY QHULL_INCLUDE_DIR)

    # ##############################################################################################
    # Create Target
    # ##############################################################################################

    if(QHULL_FOUND)
        add_library(Qhull::qhull_r UNKNOWN IMPORTED)
        set_target_properties(
            Qhull::qhull_r PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${QHULL_INCLUDE_DIR}"
        )
        set_target_properties(
            Qhull::qhull_r PROPERTIES IMPORTED_LOCATION ${QHULL_LIBRARY}
        )

        set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
        set(QHULL_LIBRARIES Qhull::qhull_r )
    endif()
else()
    if(MSVC)
        if(TARGET Qhull::qhullstatic_r)
            set(QHULL_LIBRARIES Qhull::qhullstatic_r)
        elseif(TARGET qhull::qhullstatic_r)
            set(QHULL_LIBRARIES qhull::qhullstatic_r)
        else()
            message(FATAL_ERROR "Unrecognized qhull library")
        endif()
    else()
        if(TARGET Qhull::qhull_r)
            set(QHULL_LIBRARIES Qhull::qhull_r)
        elseif(TARGET qhull::qhull_r)
            set(QHULL_LIBRARIES qhull::qhull_r)
        else()
            message(FATAL_ERROR "Unrecognized qhull library")
        endif()
    endif()
    get_target_property(QHULL_INCLUDE_DIRS ${QHULL_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
endif()
