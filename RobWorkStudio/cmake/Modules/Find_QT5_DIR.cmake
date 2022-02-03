
set(QT_MISSING True)
# msvc only; mingw will need different logic
if(MSVC)
    # look for user-registry pointing to qtcreator
    get_filename_component(
        QT_BIN
        [HKEY_CURRENT_USER\\Software\\Classes\\Applications\\QtProject.QtCreator.cpp\\shell\\Open\\Command]
        PATH
    )
    if(EXISTS "$ENV{Qt5_DIR}" AND NOT "$ENV{Qt5_DIR}" STREQUAL "/")
        set(QT_PATH "$ENV{Qt5_DIR}")
    elseif(EXISTS "${QT_BIN}" AND NOT "${QT_BIN}" STREQUAL "/")
        # get root path so we can search for 5.3, 5.4, 5.5, etc
        string(REPLACE "/Tools" ";" QT_BIN "${QT_BIN}")
        list(GET QT_BIN 0 QT_BIN)

        file(GLOB QT_VERSIONS "${QT_BIN}/5.*")
        list(SORT QT_VERSIONS)
        # assume the latest version will be last alphabetically
        list(REVERSE QT_VERSIONS)
        list(GET QT_VERSIONS 0 QT_VERSION)
        # fix any double slashes which seem to be common
        string(REPLACE "//" "/" QT_VERSION "${QT_VERSION}")

        if(CMAKE_SYSTEM_PROCESSOR MATCHES 64)
            set(BIT_SELECT "_64")
        endif()

        if(EXISTS ${QT_VERSIONS}/msvc2019${BITS})
            set(QT_PATH "${QT_VERSIONS}/msvc2019${BITS}")
        elseif(EXISTS ${QT_VERSIONS}/msvc2017${BITS})
            set(QT_PATH "${QT_VERSIONS}/msvc2017${BITS}")
        else()
            file(GLOB QT_DIR "${QT_VERSIONS}/*")
            list(SORT QT_DIR)
            list(REVERSE QT_DIR)
            string(REPLACE "//" "/" QT_DIR "${QT_DIR}")

            foreach(dir ${QT_DIR})
                if(IS_DIRECTORY ${dir})
                    set(QT_PATH "${dir}")
                endif()
            endforeach()
            
        endif()
    endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(QT_PATH "/usr/local/opt/qt5")
endif()

if(EXISTS "${QT_PATH}")
    set(QT_MISSING False)
endif()

macro(FIND_QT_PACKAGE _name)
    if(NOT QT_MISSING)
        set(REQ_QUIET)
        set(${_name}_DIR "${QT_PATH}/lib/cmake/${_name}")

        if(${${_name}_FIND_REQUIRED})
            set(REQ_QUIET REQUIRED)
        elseif(${${_name}_FIND_QUIETLY})
            set(REQ_QUIET QUIET)
        endif()
        # message(STATUS "${_name}_FIND_REQUIRED ${${_name}_FIND_REQUIRED}") message(STATUS
        # "${_name}_FIND_QUIETLY ${${_name}_FIND_QUIETLY}") message(STATUS "${_name}_FIND_VERSION
        # ${${_name}_FIND_VERSION}") message(STATUS "${_name}_FIND_VERSION_MAJOR
        # ${${_name}_FIND_VERSION_MAJOR}") message(STATUS "${_name}_FIND_VERSION_MINOR
        # ${${_name}_FIND_VERSION_MINOR}") message(STATUS "${_name}_FIND_VERSION_PATCH
        # ${${_name}_FIND_VERSION_PATCH}") message(STATUS "${_name}_FIND_VERSION_TWEAK
        # ${${_name}_FIND_VERSION_TWEAK}") message(STATUS "${_name}_FIND_VERSION_COUNT
        # ${${_name}_FIND_VERSION_COUNT}") message(STATUS "${_name}_FIND_VERSION_EXACT
        # ${${_name}_FIND_VERSION_EXACT}") message(STATUS "${_name}_FIND_COMPONENTS
        # ${${_name}_FIND_COMPONENTS}") message(STATUS "${_name}_FIND_REQUIRED_
        # ${${_name}_FIND_REQUIRED_<c>}")

        find_package(${_name} ${REQ_QUIET} PATHS "${${_name}_DIR}")
    else()
        set(CMP "${CMAKE_MODULE_PATH}")
        set(CMAKE_MODULE_PATH)
        find_package(${_name} ${REQ_QUIET})
        set(CMAKE_MODULE_PATH "${CMP}")
    endif()
endmacro()
