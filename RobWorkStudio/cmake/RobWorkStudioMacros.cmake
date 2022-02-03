# ##################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs
# to. ARGN The source files for the library.
macro(RWS_ADD_PLUGIN _name _lib_type)

    set(options EXCLUDE_FROM_ALL USING_SWIG) # Used to marke flags
    set(oneValueArgs EXPORT_SET) # used to marke values with a single value
    set(multiValueArgs)
    cmake_parse_arguments(PL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${PL_EXPORT_SET}" STREQUAL "")
        set(PL_EXPORT_SET ${PROJECT_PREFIX}Targets)
    endif()

    add_library(${_name} ${_lib_type} ${PL_UNPARSED_ARGUMENTS})
    add_dependencies(${_name} sdurws)
    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()

    if(${_lib_type} STREQUAL "MODULE" OR ${_lib_type} STREQUAL "SHARED")
        set_target_properties(
            ${_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/plugins
        )
    endif()

    # Set the VERSION and SOVERSION of the library to the RobWorkStudio major and minor versions On
    # MAC OS we can not do this if we are building a Module (where it does not make much sense
    # anyway)
    if(NOT ("${_lib_type}" STREQUAL "MODULE" AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
        string(REGEX MATCHALL "[0-9]+" VERSIONS ${ROBWORKSTUDIO_VERSION})
        list(GET VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
        list(GET VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
        list(GET VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

        set_target_properties(
            ${_name}
            PROPERTIES VERSION ${ROBWORKSTUDIO_VERSION} SOVERSION
                       ${ROBWORKSTUDIO_VERSION_MAJOR}.${ROBWORKSTUDIO_VERSION_MINOR}
                       # DEFINE_SYMBOL "RWAPI_EXPORTS"
        )
    endif()

    if(PL_EXCLUDE_FROM_ALL OR (PL_USING_SWIG AND NOT SWIG_DEFAULT_COMPILE))
        set_target_properties(${_name} PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    endif()

    if(NOT PL_USING_SWIG
       OR SWIG_DEFAULT_COMPILE
       OR CMAKE_VERSION VERSION_GREATER 3.16.0
    )
        install(
            TARGETS ${_name}
            EXPORT ${PL_EXPORT_SET}
            RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
            LIBRARY DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
            ARCHIVE DESTINATION "${LIB_INSTALL_DIR}/RobWork/rwsplugins" COMPONENT ${_component}
        )
    endif()

endmacro()

# ##################################################################################################
# Add a library target. _name The library name. _component The part of RW that this library belongs
# to. ARGN The source files for the library.
macro(RWS_ADD_COMPONENT _name)
    set(options EXCLUDE_FROM_ALL USING_SWIG) # Used to marke flags
    set(oneValueArgs EXPORT_SET) # used to marke values with a single value
    set(multiValueArgs)
    cmake_parse_arguments(PL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${PL_EXPORT_SET}" STREQUAL "")
        set(PL_EXPORT_SET ${PROJECT_PREFIX}Targets)
    endif()

    add_library(${_name} ${PROJECT_LIB_TYPE} ${PL_UNPARSED_ARGUMENTS})

    # Only link if needed
    if(WIN32 AND MSVC)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
        set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
    endif()
    #
    string(REGEX MATCHALL "[0-9]+" VERSIONS ${ROBWORKSTUDIO_VERSION})
    list(GET VERSIONS 0 PROJECT_VERSION_MAJOR)
    list(GET VERSIONS 1 PROJECT_VERSION_MINOR)
    list(GET VERSIONS 2 PROJECT_VERSION_PATCH)

    if(${PROJECT_USE_SONAME})
        set_target_properties(
            ${_name} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION
                                ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
    
    if(PL_EXCLUDE_FROM_ALL OR (PL_USING_SWIG AND NOT SWIG_DEFAULT_COMPILE))
        set_target_properties(${_name} PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    endif()

    if(NOT PL_USING_SWIG
       OR SWIG_DEFAULT_COMPILE
       OR CMAKE_VERSION VERSION_GREATER 3.16.0
    )
    install(
        TARGETS ${_name}
        EXPORT ${PL_EXPORT_SET}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_name}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_name}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_name}
    )
    endif()

endmacro()

macro(RWS_PLUGIN_LOAD_DETAILS _subsys_name _dockarea _name _visible)
    set(${_name}_DOCKAREA
        "${_name}\\DockArea=${_dockarea}"
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_FILENAME
        "${_name}\\Filename=$<TARGET_FILE_NAME:${_subsys_name}>"
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_PATH
        "${_name}\\Path=$<TARGET_FILE_DIR:${_subsys_name}>"
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_VISIBLE
        "${_name}\\Visible=${_visible}"
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
endmacro()

macro(RWS_CLEAR_PLUGIN_LOAD_DETAILS _name)
    set(${_name}_DOCKAREA
        ""
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_FILENAME
        ""
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_PATH
        ""
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
    set(${_name}_VISIBLE
        ""
        CACHE INTERNAL "PLugin Load details for {_name}" FORCE
    )
endmacro()
