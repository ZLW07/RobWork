#####################################
# only message if this is an install
function(DEBUG_MESSAGE text)
    if(DEBUG)
        message(STATUS ${text})
    endif()
endfunction()

#####################################
# Find buildtype of project and correct it if wrong assumption is made

macro(GET_RobWorkStudio_BUILD_TYPE CFG_ROOT RWS_BUILD_TYPE)
    # defaults to release
    set(BTYPE_TMP release)
    if(CMAKE_BUILD_TYPE)
        string(TOLOWER ${CMAKE_BUILD_TYPE} BTYPE_TMP)
    else()
        set(BTYPE_TMP "none")
    endif()

    # first test if the correct cmake build type is installed
    if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_${BTYPE_TMP}.cmake)
        set(${RWS_BUILD_TYPE} ${BTYPE_TMP})
    else()

        # find best RobWorkStudio build match
        if(${BTYPE_TMP} STREQUAL "release")
            # find release compatible RobWorkStudio installation
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_minsizerel.cmake)
                set(${RWS_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_relwithdebinfo.cmake)
                set(${RWS_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_debug.cmake)
                set(${RWS_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_none.cmake)
                set(${RWS_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "minsizerel")
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_release.cmake)
                set(${RWS_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_relwithdebinfo.cmake)
                set(${RWS_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_debug.cmake)
                set(${RWS_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_none.cmake)
                set(${RWS_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "relwithdebinfo")
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_release.cmake)
                set(${RWS_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_minsizerel.cmake)
                set(${RWS_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_debug.cmake)
                set(${RWS_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_none.cmake)
                set(${RWS_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "debug")
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_relwithdebinfo.cmake)
                set(${RWS_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_minsizerel.cmake)
                set(${RWS_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_release.cmake)
                set(${RWS_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_none.cmake)
                set(${RWS_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "none")
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_relwithdebinfo.cmake)
                set(${RWS_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_minsizerel.cmake)
                set(${RWS_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_debug.cmake)
                set(${RWS_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_release.cmake)
                set(${RWS_BUILD_TYPE} release)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        else()
            message(
                STATUS
                    "Does not recognize build type: ${CMAKE_BUILD_TYPE} choosing any existing RobWorkStudio installation!"
            )
            if(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_release.cmake)
                set(${RWS_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_minsizerel.cmake)
                set(${RWS_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_relwithdebinfo.cmake)
                set(${RWS_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_debug.cmake)
                set(${RWS_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkStudioBuildConfig_none.cmake)
                set(${RWS_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWorkStudio!")
            endif()
        endif()

        message(
            STATUS
                "warning: RobWorkStudio was not compiled with type:${BTYPE_TMP} using type:${${RobWorkStudio_BUILD_TYPE}} instead!"
        )
    endif()

endmacro()

#####################################
# Checks if this configuration of RobWorkStudio was installed by the windows exe installer
function(RWS_IS_WINDOWS_INSTALL_BY_EXE CFG_ROOT return)
    if(EXISTS ${CFG_ROOT}/../../RWS_uninstall.exe)
        set(${return} TRUE PARENT_SCOPE)
    else()
        set(${return} FALSE PARENT_SCOPE)
    endif()
endfunction()

#######################################
# Correct the include dirs of a target
macro(RWS_CORRECT_TARGET_INCLUDE_DIRS target)
    get_target_property(include_dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
    # message(STATUS "TARGET: ${target} input ${include_dirs}")
    if(include_dirs)
        foreach(dir ${include_dirs})

            if(NOT EXISTS ${dir})
                if("${dir}" MATCHES "boost")
                    if(EXISTS "${RWS_INCLUDE_EXT}/boost")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RWS_INCLUDE_EXT}/boost")
                        string(REPLACE "${dir}" "${RWS_INCLUDE_EXT}/boost" include_dirs "${include_dirs}")
                    endif()
                endif()
                if("${dir}" MATCHES "eigen3")
                    if(EXISTS "${RWS_INCLUDE_EXT}/Eigen")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RWS_INCLUDE_EXT}/Eigen")
                        string(REPLACE "${dir}" "${RWS_INCLUDE_EXT}/Eigen" include_dirs "${include_dirs}")
                    endif()
                endif()
                if("${dir}" MATCHES "XercesC")
                    if(EXISTS "${RWS_INCLUDE_EXT}/xerces")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RWS_INCLUDE_EXT}/xerces")
                        string(REPLACE "${dir}" "${RWS_INCLUDE_EXT}/xerces" include_dirs "${include_dirs}")
                    endif()
                endif()
            endif()
        endforeach()
        #message(STATUS "output ${include_dirs}")
        set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${include_dirs}")
    endif()

    
endmacro()

#######################################
# Correct the dependencies of a target
macro(RWS_CORRECT_TARGET_DEPENDENCIES target)
    get_target_property(link_libs ${target} INTERFACE_LINK_LIBRARIES)
    #message(STATUS "input ${link_libs}")
    if(link_libs)
        foreach(link ${link_libs})
            if(NOT EXISTS ${link})
                if("${link}" MATCHES "xerces-c")
                    FILE(GLOB _lib "${RWS_LIBS}/xerces-c*")
                    if(EXISTS "${_lib}")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RWS_INCLUDE_EXT}/xerces")
                        string(REPLACE "${link}" "${_lib}" link_libs "${link_libs}")
                    endif()
                endif()
            endif()
        endforeach()
        #message(STATUS "output ${link_libs}")
        set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${link_libs}")
    endif()
endmacro()

#####################################
# Varify if a library exists
function(RWS_VERIFY_LIBRARY _name _result)
    if(NOT DEFINED DEBUG)
        set(DEBUG FALSE)
    endif()

    set(result "")
    string(REPLACE "${RWS_BUILD_WITH_BUILD_DIR}/libs/${RWS_BUILD_TYPE}" "${RWS_LIBS}" DIRS
                   "${RWS_BUILD_WITH_LIBRARY_DIRS}"
    )
    string(REPLACE "//" "/" _name "${_name}")
    string(REPLACE ">" "" _name "${_name}")
    
    list(REMOVE_DUPLICATES DIRS)
    unset(tmp CACHE)
    find_library(
        tmp
        NAMES ${_name} lib${_name}
        PATHS ${DIRS}
        PATH_SUFFIXES ${RWS_BUILD_WITH_LIBRARY_SUBDIRS}
        NO_DEFAULT_PATH
    )

    if(${_name} MATCHES "^\\s?-l")
        set(result ${_name})

    elseif(EXISTS ${_name})
        set(result ${_name})

    elseif(TARGET ${_name})
        set(result ${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET RW::${_name})
        set(result RW::${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurw_${_name})
        set(result sdurw_${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET RW::sdurw_${_name})
        set(result RW::sdurw_${_name})
        RWS_verify_target(${result} result ${ARGN})

        elseif(TARGET RWS::${_name})
        set(result RWS::${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurws_${_name})
        set(result sdurws_${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET RWS::sdurws_${_name})
        set(result RWS::sdurws_${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET RWSIM::${_name})
        set(result RWSIM::${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurwsim_${_name})
        set(result sdurwsim_${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(TARGET RWSIM::sdurwsim_${_name})
        set(result RWSIM::sdurwsim_${_name})
        RWS_verify_target(${result} result ${ARGN})

    elseif(tmp)
        add_library(RWS::${_name} UNKNOWN IMPORTED)
        set_target_properties(
            RWS::${_name} PROPERTIES IMPORTED_LOCATION ${tmp} INTERFACE_INCLUDE_DIRECTORIES
                                    "${RWS_BUILD_WITH_INCLUDE_DIR}"
        )
        set(result "RWS::${_name}")
    else()
        unset(tmp)
        find_library(
            tmp
            NAMES ${_name} lib${_name}
        )
        if(tmp)
            add_library(${_name} UNKNOWN IMPORTED)
            set_target_properties(
                ${_name} PROPERTIES IMPORTED_LOCATION ${tmp}
            )
            set(result "${_name}")
        elseif(EXISTS ${_name})
            set(result "${_name}")
        else()
            DEBUG_MESSAGE("Could not find library: ${_name}")
        
        endif()
    endif()

    set(varified_targets
        ${varified_targets}
        PARENT_SCOPE
    )
    set(invalid_targets
        ${invalid_targets}
        PARENT_SCOPE
    )
    set(${_result}
        ${result}
        PARENT_SCOPE
    )
endfunction()

#####################################
# Varify a target and check all it's dependencies
function(RWS_VERIFY_TARGET __target __result)
    if(NOT DEFINED DEBUG)
        set(DEBUG FALSE)
    endif()
    set(idx -2)
    set(idx_u -2)
    set(idx_i -2)
    if(TARGET ${__target})
        set(result ${__target})
        list(FIND varified_targets ${__target} idx)
        list(FIND invalid_targets ${__target} idx_i)
        list(FIND ARGN ${__target} idx_u)
        if(${idx} EQUAL -1 AND ${idx_u} EQUAL -1 AND ${idx_i} EQUAL -1)
            DEBUG_MESSAGE("${__target}")
            set(underprocess ${ARGN} ${result})

            # ######################################################################################
            # Varifying underlying Binary
            # ######################################################################################
            get_target_property(type ${__target} TYPE)
            if(NOT ("${type}" STREQUAL "INTERFACE_LIBRARY"))
                get_target_property(out ${__target} IMPORTED_LOCATION_RELEASE)

                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION_RELWITHDEBINFO)
                endif()
                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION_DEBUG)
                endif()
                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION_MINSIZEREL)
                endif()
                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION_NOCONFIG)
                endif()
                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION_NONE)
                endif()
                if(NOT EXISTS ${out})
                    get_target_property(out ${__target} IMPORTED_LOCATION)
                endif()

                if(NOT EXISTS ${out})
                    set(result "")
                    DEBUG_MESSAGE(" - missing binary ${out}")
                    DEBUG_MESSAGE(" - Target Binary not found: ${__target}")
                endif()

            endif()

            # ######################################################################################
            # Varifying include dirs
            # ######################################################################################
            if(result)
                RWS_CORRECT_TARGET_INCLUDE_DIRS(${__target})
                get_target_property(out ${__target} INTERFACE_INCLUDE_DIRECTORIES)
                if(out)
                    foreach(dir ${out})

                        if(NOT EXISTS ${dir})
                            DEBUG_MESSAGE(" - Include not found: ${dir}")
                            set(result "")
                        endif()
                    endforeach()

                    if(NOT result)
                        DEBUG_MESSAGE(" - Target Include not found: ${__target}")
                    endif()
                endif()
            endif()

            # ######################################################################################
            # Varifying dependencies
            # ######################################################################################
            if(result)
                RWS_CORRECT_TARGET_DEPENDENCIES(${__target})
                get_target_property(out ${__target} INTERFACE_LINK_LIBRARIES)
                if(out)
                    foreach(lib_t ${out})
                        string(FIND ${lib_t} "\$<" found)
                        if(${found} EQUAL -1)
                            DEBUG_MESSAGE(" - Checking dep ${lib_t} for ${__target}")
                            RWS_verify_library(${lib_t} vaified_lib "${underprocess}")
                            if("${vaified_lib}" STREQUAL "")
                                set(result "")
                                DEBUG_MESSAGE(" - Target depend not found: ${lib_t} for ${__target}")
                                break()  
                            endif()
                        else()
                            DEBUG_MESSAGE(" - Target depend ignored: ${lib_t} for ${__target}")
                        endif()
                    endforeach()
                    if(NOT result)
                        DEBUG_MESSAGE(" - Target depend not found: ${__target}")
                    endif()
                endif()
            endif()

            # ######################################################################################
            # Saving Result
            # ######################################################################################
            if(NOT ${result} STREQUAL "")
                set(varified_targets ${varified_targets} ${result})
                DEBUG_MESSAGE(" - Target varified: ${result}")
            else()
                set(invalid_targets ${invalid_targets} ${__target})
            endif()
        elseif(${idx} GREATER -1)
            DEBUG_MESSAGE(" - Target ${__target} already verified")
        elseif(${idx_u} GREATER -1)
            DEBUG_MESSAGE(" - Target ${__target} ARGN verified")
        elseif(${idx_i} GREATER -1)
            DEBUG_MESSAGE(" - Target ${__target} Invalid target")
            set(${__result} "")
        endif()
    else()
        set(${__result} "")
    endif()

    set(invalid_targets
        ${invalid_targets}
        PARENT_SCOPE
    )
    set(varified_targets
        ${varified_targets}
        PARENT_SCOPE
    )
    set(${__result}
        ${result}
        PARENT_SCOPE
    )
endfunction()

#####################################
# only message if this is an install
function(INSTALL_MESSAGE text)
    if(IS_INSTALL)
        message(STATUS ${text})
    endif()
endfunction()

#####################################
# Setup library, external and include directories.
macro(RWS_SETUP_CONFIG_DIRECTORIES CFG_DIR)

    # check whether RobWorkStudioConfig.cmake is found into a RobWorkStudio installation or in a build tree
    if(EXISTS "${CFG_DIR}/../src/RobWorkStudioConfig.hpp")
        # Found RobWorkStudioConfig.cmake in a build tree of RobWorkStudio
        message(STATUS "RobWorkStudio: Found a RobWorkStudio build tree")
        set(RWS_ROOT "${CFG_DIR}/..")

        set(RWS_INCLUDE_EXT "${RWS_ROOT}/ext")
        set(RWS_INCLUDE_SRC "${RWS_ROOT}/src")
        set(RWS_LIBS "${RWS_BUILD_WITH_BUILD_DIR}/libs/${RWS_BUILD_TYPE}")
        set(IS_INSTALL FALSE)
    else()
        message(STATUS "RobWorkStudio: Found a RobWorkStudio installation")
        set(IS_INSTALL TRUE)
        string(TOLOWER "RobWorkStudio" ROBWORK_PROJECT_NAME_LOWERCASE)
        # Found a RobWorkStudio installation
        if(WIN32)
            # RobWorkStudioConfig.cmake is installed to RWS_ROOT/cmake
            set(RWS_ROOT "${CFG_DIR}/../..")
            set(RWS_INCLUDE_SRC
                "${RWS_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWorkStudio_VERSION_MAJOR}.${RobWorkStudio_VERSION_MINOR}"
            )
            set(RWS_INCLUDE_EXT "${RWS_INCLUDE_SRC}/ext")
            set(RWS_LIBS "${RWS_ROOT}/lib")
        else()
            # RobWorkStudioConfig.cmake is installed to RWS_ROOT/share/rRobWorkStudio-x.y/cmake
            set(RWS_ROOT "${CFG_DIR}/../../..")
            set(RWS_LIBS "${RWS_ROOT}/lib")
            set(RWS_INCLUDE_SRC
                "${RWS_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWorkStudio_VERSION_MAJOR}.${RobWorkStudio_VERSION_MINOR}"
            )
            set(RWS_INCLUDE_EXT
                "${RWS_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWorkStudio_VERSION_MAJOR}.${RobWorkStudio_VERSION_MINOR}/ext"
            )
        endif()
    endif()
endmacro()

######################################
# Find required components or include all found components
function(RWS_FIND_COMPONENTS _return)
    string(TOUPPER "RobWorkStudio" ROBWORK_PROJECT_NAME_UPPERCASE)
    string(TOLOWER "RWS" PROJECT_PREFIX_LOWERCASE)
    set(${_return})
    if(RobWorkStudio_FIND_COMPONENTS)
        # FIRST check if all required components are installed/build
        set(LIBRARIES_TO_INCLUDE RWS::sdu${PROJECT_PREFIX_LOWERCASE})
        foreach(
            component
            IN
            LISTS RobWorkStudio_FIND_COMPONENTS
        )
            if(NOT (${component} STREQUAL "sdu${PROJECT_PREFIX_LOWERCASE}"))
                unset(res)
                verify_library(${component} res)

                if(NOT "${res}" STREQUAL "")
                    list(APPEND ${_return} "${res}")
                    set(RobWorkStudio_${component}_FOUND TRUE)
                else()
                    set(RobWorkStudio${component}_FOUND FALSE)
                    if(RobWorkStudio_REQUIRED)
                        set(RobWorkStudio_EXPERIENCED_FATAL_PROBLEMS TRUE)
                        message(
                            FATAL_ERROR
                                "The component: sdu${PROJECT_PREFIX_LOWERCASE}_${component} has not been built with RobWorkStudio. Reconfigure RobWorkStudio installation or check component spelling!"
                        )
                    else()
                        set(RobWorkStudio_EXPERIENCED_FATAL_PROBLEMS TRUE)
                        message(
                            WARNING
                                "The component: sdu${PROJECT_PREFIX_LOWERCASE}_${component} has not been built with RobWorkStudio. Reconfigure RobWorkStudio installation or check component spelling!"
                        )
                    endif()
                endif()
            endif()
        endforeach()
    else()
        foreach(
            lib
            IN
            LISTS RWS_BUILD_WITH_LIBRARIES
        )
            unset(res)
            RWS_verify_library(${lib} res)
            if(NOT "${res}" STREQUAL "")
                list(APPEND ${_return} "${res}")
                INSTALL_MESSAGE("Looking for ${lib} - found")
            else()
                if(${lib} STREQUAL "sdu${PROJECT_PREFIX_LOWERCASE}" AND RobWorkStudio_REQUIRED)
                    list(APPEND ${_return} "RWS::${lib}")
                    message(FATAL_ERROR "Looking for ${lib} - not found" )
                else()
                    INSTALL_MESSAGE("Looking for ${lib} - not found (ignored)")
                endif()
            endif()
        endforeach()
    endif()

    set(${_return} ${${_return}} PARENT_SCOPE)
endfunction()

#######################################
# Setup linker flags and build definitions
macro(RWS_SETUP_FLAGS_AND_DEFINITIONS)
    # Set extra compiler flags. The user should be able to change this
    set(RWS_C_FLAGS
    ${RWS_BUILD_WITH_C_FLAGS}
    CACHE STRING "Change this to force using your own
                flags and not those of RobWorkStudio"
    )
    set(RWS_CXX_FLAGS
        ${RWS_BUILD_WITH_CXX_FLAGS}
        CACHE STRING "Change this to force using your own
                    flags and not those of RobWorkStudio"
    )
    set(RWS_DEFINITIONS
        ${RWS_BUILD_WITH_DEFINITIONS}
        CACHE STRING "Change this to force using your own
                    definitions and not those of RobWorkStudio"
    )
    add_definitions(${RWS_DEFINITIONS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWS_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${RWS_C_FLAGS}")

    # Set extra linker flags. The user should be able to change this
    set(RWS_LINKER_FLAGS
        ${RWS_BUILD_WITH_LINKER_FLAGS}
        CACHE STRING "Change this to force using your own linker
                    flags and not those of RobWorkStudio"
    )
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} ${RWS_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    set(CMAKE_MODULE_LINKER_FLAGS
        "${CMAKE_MODULE_LINKER_FLAGS} ${RWS_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    if(WIN32)
        set(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} ${RWS_LINKER_FLAGS}"
            CACHE STRING "" FORCE
        )
    endif()
endmacro()

#######################################
# Generate boost targets for windows exe installs
macro(RWS_GENERATE_BOOST_EXE_TARGETS)
    foreach(pkg filesystem serialization system thread program_options)
        if (NOT TARGET Boost::${pkg})
            FILE(GLOB boostlib "${RWS_LIBS}/libboost_${pkg}-*")
            add_library(Boost::${pkg} UNKNOWN IMPORTED)
            set_target_properties(Boost::${pkg}  PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${RWS_INCLUDE_EXT}/boost
                IMPORTED_LOCATION ${boostlib}
            )
        endif()
    endforeach()
    set(Boost_FOUND TRUE)
endmacro()
