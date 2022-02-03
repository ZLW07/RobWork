# Check whether the requested PACKAGE_FIND_VERSION is compatible
set(PACKAGE_VERSION 22.2.3)
set(PACKAGE_GOT_VERSION False)

# Check whether the requested PACKAGE_FIND_VERSION is compatible
if(${PACKAGE_GOT_VERSION})
    if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
        set(PACKAGE_VERSION_COMPATIBLE FALSE)
    else()
        set(PACKAGE_VERSION_COMPATIBLE TRUE)
        if ("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
            set(PACKAGE_VERSION_EXACT TRUE)
        endif()
    endif()
else()
    message(STATUS "RobWorkSim has not been compiled with a Version Number")
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
endif()
