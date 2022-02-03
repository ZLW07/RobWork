# Find FCL using pkg-config files
include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(FCL fcl)
endif()
if(FCL_FOUND)
    set(FCL_INCLUDE_DIRS ${FCL_INCLUDEDIR})
    if("${FCL_LIBRARY_DIRS}" STREQUAL "")
        set(FCL_LIBRARY_DIRS ${FCL_LIBDIR})
    endif()
else(FCL_FOUND)
    set(FCL_INCLUDE_DIRS)
    set(FCL_LIBRARY_DIRS)
    set(FCL_LIBRARIES)
    set(FCL_VERSION)
endif(FCL_FOUND)
find_package_handle_standard_args(
    FCL
    FOUND_VAR
    FCL_FOUND
    REQUIRED_VARS
    FCL_LIBRARIES
    FCL_INCLUDE_DIRS
    FCL_LIBRARY_DIRS
    VERSION_VAR
    FCL_VERSION
)
