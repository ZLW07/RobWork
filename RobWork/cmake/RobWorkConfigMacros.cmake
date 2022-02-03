#####################################
# only message if this is an install
function(DEBUG_MESSAGE text)
    if(DEBUG)
        message(STATUS ${text})
    endif()
endfunction()

#####################################
# Find buildtype of project and correct it if wrong assumption is made

macro(GET_RobWork_BUILD_TYPE CFG_ROOT RW_BUILD_TYPE)
    # defaults to release
    set(BTYPE_TMP release)
    if(CMAKE_BUILD_TYPE)
        string(TOLOWER ${CMAKE_BUILD_TYPE} BTYPE_TMP)
    else()
        set(BTYPE_TMP "none")
    endif()

    # first test if the correct cmake build type is installed
    if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_${BTYPE_TMP}.cmake)
        set(${RW_BUILD_TYPE} ${BTYPE_TMP})
    else()

        # find best RobWork build match
        if(${BTYPE_TMP} STREQUAL "release")
            # find release compatible RobWork installation
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake)
                set(${RW_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_relwithdebinfo.cmake)
                set(${RW_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_debug.cmake)
                set(${RW_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_none.cmake)
                set(${RW_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "minsizerel")
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_release.cmake)
                set(${RW_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_relwithdebinfo.cmake)
                set(${RW_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_debug.cmake)
                set(${RW_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_none.cmake)
                set(${RW_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "relwithdebinfo")
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_release.cmake)
                set(${RW_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake)
                set(${RW_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_debug.cmake)
                set(${RW_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_none.cmake)
                set(${RW_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "debug")
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_relwithdebinfo.cmake)
                set(${RW_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake)
                set(${RW_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_release.cmake)
                set(${RW_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_none.cmake)
                set(${RW_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        elseif(${BTYPE_TMP} STREQUAL "none")
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_relwithdebinfo.cmake)
                set(${RW_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake)
                set(${RW_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_debug.cmake)
                set(${RW_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_release.cmake)
                set(${RW_BUILD_TYPE} release)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        else()
            message(
                STATUS
                    "Does not recognize build type: ${CMAKE_BUILD_TYPE} choosing any existing RobWork installation!"
            )
            if(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_release.cmake)
                set(${RW_BUILD_TYPE} release)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_minsizerel.cmake)
                set(${RW_BUILD_TYPE} minsizerel)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_relwithdebinfo.cmake)
                set(${RW_BUILD_TYPE} relwithdebinfo)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_debug.cmake)
                set(${RW_BUILD_TYPE} debug)
            elseif(EXISTS ${CFG_ROOT}/RobWorkBuildConfig_none.cmake)
                set(${RW_BUILD_TYPE} none)
            else()
                message(FATAL_ERROR "Could not find any build of RobWork!")
            endif()
        endif()

        message(
            STATUS
                "warning: RobWork was not compiled with type:${BTYPE_TMP} using type:${${RobWork_BUILD_TYPE}} instead!"
        )
    endif()

endmacro()

#####################################
# Checks if this configuration of RobWork was installed by the windows exe installer
function(RW_IS_WINDOWS_INSTALL_BY_EXE CFG_ROOT return)
    if(EXISTS ${CFG_ROOT}/../../RW_uninstall.exe)
        set(${return} TRUE PARENT_SCOPE)
    else()
        set(${return} FALSE PARENT_SCOPE)
    endif()
endfunction()

#######################################
# Correct the include dirs of a target
macro(RW_CORRECT_TARGET_INCLUDE_DIRS target)
    get_target_property(include_dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
    # message(STATUS "TARGET: ${target} input ${include_dirs}")
    if(include_dirs)
        foreach(dir ${include_dirs})

            if(NOT EXISTS ${dir})
                if("${dir}" MATCHES "boost")
                    if(EXISTS "${RW_INCLUDE_EXT}/boost")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RW_INCLUDE_EXT}/boost")
                        string(REPLACE "${dir}" "${RW_INCLUDE_EXT}/boost" include_dirs "${include_dirs}")
                    endif()
                endif()
                if("${dir}" MATCHES "eigen3")
                    if(EXISTS "${RW_INCLUDE_EXT}/Eigen")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RW_INCLUDE_EXT}/Eigen")
                        string(REPLACE "${dir}" "${RW_INCLUDE_EXT}/Eigen" include_dirs "${include_dirs}")
                    endif()
                endif()
                if("${dir}" MATCHES "XercesC")
                    if(EXISTS "${RW_INCLUDE_EXT}/xerces")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RW_INCLUDE_EXT}/xerces")
                        string(REPLACE "${dir}" "${RW_INCLUDE_EXT}/xerces" include_dirs "${include_dirs}")
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
macro(RW_CORRECT_TARGET_DEPENDENCIES target)
    get_target_property(link_libs ${target} INTERFACE_LINK_LIBRARIES)
    #message(STATUS "input ${link_libs}")
    if(link_libs)
        foreach(link ${link_libs})
            if(NOT EXISTS ${link})
                if("${link}" MATCHES "xerces-c")
                    FILE(GLOB _lib "${RW_LIBS}/xerces-c*")
                    if(EXISTS "${_lib}")
                        #message(STATUS "target ${target} replaces: ${dir} with ${RW_INCLUDE_EXT}/xerces")
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
function(RW_VERIFY_LIBRARY _name _result)
    if(NOT DEFINED DEBUG)
        set(DEBUG FALSE)
    endif()

    set(result "")
    string(REPLACE "${RW_BUILD_WITH_BUILD_DIR}/libs/${RW_BUILD_TYPE}" "${RW_LIBS}" DIRS
                   "${RW_BUILD_WITH_LIBRARY_DIRS}"
    )
    string(REPLACE "//" "/" _name "${_name}")
    string(REPLACE ">" "" _name "${_name}")
    
    list(REMOVE_DUPLICATES DIRS)
    unset(tmp CACHE)
    find_library(
        tmp
        NAMES ${_name} lib${_name}
        PATHS ${DIRS}
        PATH_SUFFIXES ${RW_BUILD_WITH_LIBRARY_SUBDIRS}
        NO_DEFAULT_PATH
    )

    if(${_name} MATCHES "^\\s?-l")
        set(result ${_name})

    elseif(EXISTS ${_name})
        set(result ${_name})

    elseif(TARGET ${_name})
        set(result ${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET RW::${_name})
        set(result RW::${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurw_${_name})
        set(result sdurw_${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET RW::sdurw_${_name})
        set(result RW::sdurw_${_name})
        RW_verify_target(${result} result ${ARGN})

        elseif(TARGET RWS::${_name})
        set(result RWS::${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurws_${_name})
        set(result sdurws_${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET RWS::sdurws_${_name})
        set(result RWS::sdurws_${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET RWSIM::${_name})
        set(result RWSIM::${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET sdurwsim_${_name})
        set(result sdurwsim_${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(TARGET RWSIM::sdurwsim_${_name})
        set(result RWSIM::sdurwsim_${_name})
        RW_verify_target(${result} result ${ARGN})

    elseif(tmp)
        add_library(RW::${_name} UNKNOWN IMPORTED)
        set_target_properties(
            RW::${_name} PROPERTIES IMPORTED_LOCATION ${tmp} INTERFACE_INCLUDE_DIRECTORIES
                                    "${RW_BUILD_WITH_INCLUDE_DIR}"
        )
        set(result "RW::${_name}")
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
function(RW_VERIFY_TARGET __target __result)
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
                RW_CORRECT_TARGET_INCLUDE_DIRS(${__target})
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
                RW_CORRECT_TARGET_DEPENDENCIES(${__target})
                get_target_property(out ${__target} INTERFACE_LINK_LIBRARIES)
                if(out)
                    foreach(lib_t ${out})
                        string(FIND ${lib_t} "\$<" found)
                        if(${found} EQUAL -1)
                            DEBUG_MESSAGE(" - Checking dep ${lib_t} for ${__target}")
                            RW_verify_library(${lib_t} vaified_lib "${underprocess}")
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
macro(RW_SETUP_CONFIG_DIRECTORIES CFG_DIR)

    # check whether RobWorkConfig.cmake is found into a RobWork installation or in a build tree
    if(EXISTS "${CFG_DIR}/../src/RobWorkConfig.hpp")
        # Found RobWorkConfig.cmake in a build tree of RobWork
        message(STATUS "RobWork: Found a RobWork build tree")
        set(RW_ROOT "${CFG_DIR}/..")

        set(RW_INCLUDE_EXT "${RW_ROOT}/ext")
        set(RW_INCLUDE_SRC "${RW_ROOT}/src")
        set(RW_LIBS "${RW_BUILD_WITH_BUILD_DIR}/libs/${RW_BUILD_TYPE}")
        set(IS_INSTALL FALSE)
    else()
        message(STATUS "RobWork: Found a RobWork installation")
        set(IS_INSTALL TRUE)
        string(TOLOWER "RobWork" ROBWORK_PROJECT_NAME_LOWERCASE)
        # Found a RobWork installation
        if(WIN32)
            # RobWorkConfig.cmake is installed to RW_ROOT/cmake
            set(RW_ROOT "${CFG_DIR}/../..")
            set(RW_INCLUDE_SRC
                "${RW_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWork_VERSION_MAJOR}.${RobWork_VERSION_MINOR}"
            )
            set(RW_INCLUDE_EXT "${RW_INCLUDE_SRC}/ext")
            set(RW_LIBS "${RW_ROOT}/lib")
        else()
            # RobWorkConfig.cmake is installed to RW_ROOT/share/rRobWork-x.y/cmake
            set(RW_ROOT "${CFG_DIR}/../../..")
            set(RW_LIBS "${RW_ROOT}/lib")
            set(RW_INCLUDE_SRC
                "${RW_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWork_VERSION_MAJOR}.${RobWork_VERSION_MINOR}"
            )
            set(RW_INCLUDE_EXT
                "${RW_ROOT}/include/${ROBWORK_PROJECT_NAME_LOWERCASE}-${RobWork_VERSION_MAJOR}.${RobWork_VERSION_MINOR}/ext"
            )
        endif()
    endif()
endmacro()

######################################
# Find required components or include all found components
function(RW_FIND_COMPONENTS _return)
    string(TOUPPER "RobWork" ROBWORK_PROJECT_NAME_UPPERCASE)
    string(TOLOWER "RW" PROJECT_PREFIX_LOWERCASE)
    set(${_return})
    if(RobWork_FIND_COMPONENTS)
        # FIRST check if all required components are installed/build
        set(LIBRARIES_TO_INCLUDE RW::sdu${PROJECT_PREFIX_LOWERCASE})
        foreach(
            component
            IN
            LISTS RobWork_FIND_COMPONENTS
        )
            if(NOT (${component} STREQUAL "sdu${PROJECT_PREFIX_LOWERCASE}"))
                unset(res)
                verify_library(${component} res)

                if(NOT "${res}" STREQUAL "")
                    list(APPEND ${_return} "${res}")
                    set(RobWork_${component}_FOUND TRUE)
                else()
                    set(RobWork${component}_FOUND FALSE)
                    if(RobWork_REQUIRED)
                        set(RobWork_EXPERIENCED_FATAL_PROBLEMS TRUE)
                        message(
                            FATAL_ERROR
                                "The component: sdu${PROJECT_PREFIX_LOWERCASE}_${component} has not been built with RobWork. Reconfigure RobWork installation or check component spelling!"
                        )
                    else()
                        set(RobWork_EXPERIENCED_FATAL_PROBLEMS TRUE)
                        message(
                            WARNING
                                "The component: sdu${PROJECT_PREFIX_LOWERCASE}_${component} has not been built with RobWork. Reconfigure RobWork installation or check component spelling!"
                        )
                    endif()
                endif()
            endif()
        endforeach()
    else()
        foreach(
            lib
            IN
            LISTS RW_BUILD_WITH_LIBRARIES
        )
            unset(res)
            RW_verify_library(${lib} res)
            if(NOT "${res}" STREQUAL "")
                list(APPEND ${_return} "${res}")
                INSTALL_MESSAGE("Looking for ${lib} - found")
            else()
                if(${lib} STREQUAL "sdu${PROJECT_PREFIX_LOWERCASE}" AND RobWork_REQUIRED)
                    list(APPEND ${_return} "RW::${lib}")
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
macro(RW_SETUP_FLAGS_AND_DEFINITIONS)
    # Set extra compiler flags. The user should be able to change this
    set(RW_C_FLAGS
    ${RW_BUILD_WITH_C_FLAGS}
    CACHE STRING "Change this to force using your own
                flags and not those of RobWork"
    )
    set(RW_CXX_FLAGS
        ${RW_BUILD_WITH_CXX_FLAGS}
        CACHE STRING "Change this to force using your own
                    flags and not those of RobWork"
    )
    set(RW_DEFINITIONS
        ${RW_BUILD_WITH_DEFINITIONS}
        CACHE STRING "Change this to force using your own
                    definitions and not those of RobWork"
    )
    add_definitions(${RW_DEFINITIONS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RW_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${RW_C_FLAGS}")

    # Set extra linker flags. The user should be able to change this
    set(RW_LINKER_FLAGS
        ${RW_BUILD_WITH_LINKER_FLAGS}
        CACHE STRING "Change this to force using your own linker
                    flags and not those of RobWork"
    )
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    set(CMAKE_MODULE_LINKER_FLAGS
        "${CMAKE_MODULE_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    if(WIN32)
        set(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
            CACHE STRING "" FORCE
        )
    endif()
endmacro()

#######################################
# Generate boost targets for windows exe installs
macro(RW_GENERATE_BOOST_EXE_TARGETS)
    foreach(pkg filesystem serialization system thread program_options)
        if (NOT TARGET Boost::${pkg})
            FILE(GLOB boostlib "${RW_LIBS}/libboost_${pkg}-*")
            add_library(Boost::${pkg} UNKNOWN IMPORTED)
            set_target_properties(Boost::${pkg}  PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${RW_INCLUDE_EXT}/boost
                IMPORTED_LOCATION ${boostlib}
            )
        endif()
    endforeach()
    set(Boost_FOUND TRUE)
endmacro()
