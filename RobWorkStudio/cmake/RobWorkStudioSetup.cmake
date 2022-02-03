# Find and sets up RobWorkStudio.
#
# ROBWORKSTUDIO_INCLUDE_DIR - Where to find robwork include sub-directory. ROBWORKSTUDIO_LIBRARIES -
# List of libraries when using RobWork (includes all libraries that RobWork depends on).
# ROBWORKSTUDIO_LIBRARY_DIRS - List of directories where libraries of RobWork are located.
# ROBWORKSTUDIO_FOUND       - True if RobWork was found. (not impl yet)
#
# RWS_ROOT             - If set this defines the root of ROBWORKSTUDIO if not set then it if
# possible be autodetected.
#

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

if(RWSTUDIO_ROOT)
    set(RWS_ROOT ${RWSTUDIO_ROOT})
endif()

# Check if RWstudio_ROOT path are setup correctly
find_file(ROBWORKSTUDIO_FOUND RobWorkStudioSetup.cmake ${RWSTUDIO_ROOT}/cmake ${RWS_ROOT}/cmake
          NO_DEFAULT_PATH
)
if(NOT ROBWORKSTUDIO_FOUND)
    message(
        SEND_ERROR
            "RobWorkStudio: Path to RobWorkStudio root (RWSTUDIO_ROOT) is incorrectly setup! \nRWSTUDIO_ROOT == ${RWSTUDIO_ROOT}"
    )
endif()
message(STATUS "RobWorkStudio: ROOT dir: ${RWS_ROOT}")

#
# Setup the default include and library dirs for RobWorkStudio
#
set(CMAKE_MODULE_PATH ${RWS_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

# ##################################################################################################
# DEPENDENCIES - REQUIRED Check for all dependencies, this adds LIBRARY_DIRS and include dirs that
# the configuration depends on
#

find_package(PythonInterp 3 QUIET)
find_package(PythonLibs 3 QUIET)
if(PYTHONLIBS_FOUND)
    set(RWS_USE_PYTHON3 true)
    set(RWS_USE_PYTHON true)
else()
    find_package(PythonLibs QUIET)
    if(PYTHONLIBS_FOUND)
        set(RWS_USE_PYTHON2 true)
        set(RWS_USE_PYTHON true)
    endif()
endif()

if(NOT PYTHONINTERP_FOUND)
    find_package(PythonInterp QUIET)
endif()

if(PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
    message(STATUS )
    if(NOT (${PYTHONLIBS_VERSION_STRING} STREQUAL ${PYTHON_VERSION_STRING}))
        string(ASCII 27 Esc)
        message(
            WARNING
                "${Esc}[33mMatching Versions of python intepretor and python library NOT FOUND. \r"
                "Found versions are python libs ${PYTHONLIBS_VERSION_STRING} and python intepretor ${PYTHON_VERSION_STRING}. \n"
                "This can be because you haven't installed python${PYTHON_VERSION_MAJOR}-dev package\n${Esc}[m"
        )
    endif()
endif()

if(PYTHONINTERP_FOUND)
    message(STATUS "Found Python interpreter ${PYTHON_VERSION_STRING}")
endif()
if(PYTHONLIBS_FOUND)
    message(STATUS "Found Python libraries ${PYTHONLIBS_VERSION_STRING}")
endif()

if(NOT PYTHON_LIBRARIES)
    set(PYTHON_LIBRARIES "")
endif()

# Find and setup OpenGL.
if(POLICY CMP0072) # Introduce cmake 3.11
    cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

# Find and setup Qt.
find_package(Qt6 COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets QUIET)
if(Qt6Core_FOUND
   AND Qt6Gui_FOUND
   AND Qt6Widgets_FOUND
   AND Qt6OpenGL_FOUND
   AND Qt6OpenGLWidgets_FOUND
)
    set(QT_LIBRARIES ${Qt6Core_LIBRARIES} ${Qt6Gui_LIBRARIES} ${Qt6Widgets_LIBRARIES}
                     ${Qt6OpenGL_LIBRARIES} ${Qt6OpenGLWidgets_LIBRARIES}
    )
    message(STATUS "RobWorkStudio: Using Qt ${Qt6_VERSION}.")
    set(CMAKE_AUTOMOC ON)
else()
    message(STATUS "RobWorkStudio: One or more Qt6 modules not found:")
    if(Qt6Core_FOUND)
        message(STATUS "RobWorkStudio: - Qt6Core found.")
    else()
        message(STATUS "RobWorkStudio: - Qt6Core NOT found. Please set Qt6Core_DIR to find.")
    endif()
    if(Qt6Gui_FOUND)
        message(STATUS "RobWorkStudio: - Qt6Gui found.")
    else()
        message(STATUS "RobWorkStudio: - Qt6Gui NOT found. Please set Qt6Gui_DIR to find.")
    endif()
    if(Qt6Widgets_FOUND)
        message(STATUS "RobWorkStudio: - Qt6Widgets found.")
    else()
        message(STATUS "RobWorkStudio: - Qt6Widgets NOT found. Please set Qt6Widgets_DIR to find.")
    endif()
    if(Qt6OpenGL_FOUND)
        message(STATUS "RobWorkStudio: - Qt6OpenGL found.")
    else()
        message(STATUS "RobWorkStudio: - Qt6OpenGL NOT found. Please set Qt6OpenGL_DIR to find.")
    endif()
    if(Qt6OpenGLWidgets_FOUND)
        message(STATUS "RobWorkStudio: - Qt6OpenGLWidgets found.")
    else()
        message(
            STATUS
                "RobWorkStudio: - Qt6OpenGLWidgets NOT found. Please set Qt6OpenGLWidgets_DIR to find."
        )
    endif()

    find_package(Qt5Core 5.5.1 QUIET)
    find_package(Qt5Gui 5.5.1 QUIET)
    find_package(Qt5Widgets 5.5.1 QUIET)
    find_package(Qt5OpenGL 5.5.1 QUIET)
    if(Qt5Core_FOUND
       AND Qt5Gui_FOUND
       AND Qt5Widgets_FOUND
       AND Qt5OpenGL_FOUND
    )
        set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES}
                         ${Qt5OpenGL_LIBRARIES}
        )
        message(STATUS "RobWorkStudio: Using Qt ${Qt5Core_VERSION}.")
        set(CMAKE_AUTOMOC ON)
    else()
        message(STATUS "RobWorkStudio: One or more Qt5 modules not found:")
        if(Qt5Core_FOUND)
            message(STATUS "RobWorkStudio: - Qt5Core found.")
        else()
            message(STATUS "RobWorkStudio: - Qt5Core NOT found. Please set Qt5Core_DIR to find.")
        endif()
        if(Qt5Gui_FOUND)
            message(STATUS "RobWorkStudio: - Qt5Gui found.")
        else()
            message(STATUS "RobWorkStudio: - Qt5Gui NOT found. Please set Qt5Gui_DIR to find.")
        endif()
        if(Qt5Widgets_FOUND)
            message(STATUS "RobWorkStudio: - Qt5Widgets found.")
        else()
            message(
                STATUS "RobWorkStudio: - Qt5Widgets NOT found. Please set Qt5Widgets_DIR to find."
            )
        endif()
        if(Qt5OpenGL_FOUND)
            message(STATUS "RobWorkStudio: - Qt5OpenGL found.")
        else()
            message(
                STATUS "RobWorkStudio: - Qt5OpenGL NOT found. Please set Qt5OpenGL_DIR to find."
            )
        endif()
        message(
            FATAL_ERROR
                "RobWorkStudio: Could NOT find Qt6 or Qt5. Please set the Qt6 or Qt5 directories."
        )
    endif()
endif()

# ##################################################################################################
# DEPENDENCIES - OPTIONAL these dependencies are optional, which is the user can switch off modules

set(RWS_HAVE_GLUT False)
set(RWS_HAVE_FREEGLUT FALSE)

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_package(GLUT QUIET)
    if(NOT GLUT_FOUND) # Check if free glut exsist
        find_package(FreeGLUT QUIET)
        if(FreeGLUT_FOUND)
            set(GLUT_glut_LIBRARY FreeGLUT::freeglut_static)
            set(GLUT_FOUND ${FreeGLUT_FOUND})
            set(RWS_HAVE_FREEGLUT TRUE)
        endif()
    endif()
endif()
if(OPENGL_FOUND AND GLUT_FOUND)
    set(RWS_HAVE_GLUT True)
    message(STATUS "RobWork: OpenGL and GLUT ENABLED! FOUND!")
else()
    set(GLUT_glut_LIBRARY "")
    message(STATUS "RobWork: OpenGL and GLUT NOT FOUND! code disabled!")
endif()

# Check if SWIG is available
if(RW_BUILD_WITH_SWIG AND NOT DEFINED SWIG_EXECUTABLE)
    set(SWIG_EXECUTABLE ${RW_BUILD_WITH_SWIG_CMD})
    set(SWIG_VERSION ${RW_BUILD_WITH_SWIG_VERSION})
endif()

find_package(SWIG 3.0.0 QUIET) # At least SWIG 3 to support C++11
if(SWIG_FOUND)
    message(STATUS "RobWorkStudio: SWIG ${SWIG_VERSION} found!")
else()
    message(STATUS "RobWorkStudio: SWIG 3+ not found!")
endif()

# optional compilation of LUA interface
include(CMakeDependentOption)
set(RWS_HAVE_LUA False)
cmake_dependent_option(
    RWS_DISABLE_LUA "Set when you want to disable lua!" OFF "RW_BUILD_WITH_LUA AND SWIG_FOUND" ON
)
if(NOT RWS_DISABLE_LUA)
    if(NOT SWIG_FOUND)
        message(STATUS "RobWorkStudio: Lua DISABLED! - SWIG 3+ was not found!")
        set(RWS_HAVE_LUA False)
    elseif(RW_BUILD_WITH_LUA)
        message(STATUS "RobWorkStudio: Lua ENABLED!")
        set(RWS_LUA "sdurws_lua_s;sdurws_luaeditor")
        set(RWS_HAVE_LUA True)
    else()
        message(STATUS "RobWorkStudio: Lua DISABLED! - RobWork is NOT compiled with Lua support!")
        set(RWS_HAVE_LUA False)
    endif()
else()
    message(STATUS "RobWorkStudio: Lua DISABLED!")
endif()

# QCodeEditor

find_package(QCodeEditor QUIET)
set(RWS_USE_QCODEEDITOR ON)
if(QCodeEditor_FOUND)
    set(RWS_QCODEEDITOR_INTERNAL_TARGET OFF)
    message("QCodeEditor found")
elseif(
    Qt6Core_FOUND
    AND Qt6Widgets_FOUND
    AND Qt6Gui_FOUND
    AND CMAKE_VERSION VERSION_GREATER 3.6
)
    set(RWS_QCODEEDITOR_INTERNAL_TARGET ON)
    message(STATUS "QCodeEditor not found building internal target")
elseif(
    Qt5Core_FOUND
    AND Qt5Widgets_FOUND
    AND Qt5Gui_FOUND
    AND CMAKE_VERSION VERSION_GREATER 3.6
)
    set(RWS_QCODEEDITOR_INTERNAL_TARGET ON)
    message(STATUS "QCodeEditor not found building internal target")
else()
    set(RWS_QCODEEDITOR_INTERNAL_TARGET OFF)
    message(STATUS "QCodeEditor not found. Internal target can't be build")
    set(RWS_USE_QCODEEDITOR OFF)
endif()

# ##################################################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Set extra compiler flags. The user should be able to change this. The compiler flags from RobWork
# are automatically set
#
rw_is_release(IS_RELEASE)

if(NOT DEFINED RWS_CXX_FLAGS)
    set(RWS_CXX_FLAGS
        "${RW_BUILD_WITH_CXX_FLAGS} ${RWS_CXX_FLAGS_TMP}"
        CACHE STRING "Change this to force using your own flags and not those of RobWorkSutdio"
    )
endif()

if(NOT DEFINED RWS_DEFINITIONS)
    if(${IS_RELEASE})
        set(RWS_DEFINITIONS_TMP "-DQT_NO_DEBUG")
    else()
        set(RWS_DEFINITIONS_TMP "-DQT_DEBUG")
    endif()

    set(RWS_DEFINITIONS
        "${RW_BUILD_WITH_DEFINITIONS};${RWS_DEFINITIONS_TMP}"
        CACHE STRING
              "Change this to force using your own definitions and not those of RobWorkSutdio"
    )
endif()

add_definitions(${RWS_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWS_CXX_FLAGS}")
message(STATUS "RobWorkStudio: Adding RWS CXX flags: ${RWS_CXX_FLAGS}")
message(STATUS "RobWorkStudio: Addubg RWS definitions: ${RWS_DEFINITIONS}")

#
# Set extra linker flags. The user should be able to change this. The linker flags from RobWork are
# automatically set.
#
if(DEFINED RWS_LINKER_FLAGS)
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} ${RWS_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
    if(WIN32)
        set(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} ${RWS_LINKER_FLAGS}"
            CACHE STRING "" FORCE
        )
    endif()

    message(STATUS "RobWorkStudio: Adding RWS linker flags: ${RWS_LINKER_FLAGS}")
endif()

# ##################################################################################################
# SETTING UP VARS here we setup the output variables
#

# Setup RobWorkStudio include and link directories
set(ROBWORKSTUDIO_INCLUDE_DIR ${RWS_ROOT}/src/)
set(ROBWORKSTUDIO_LIBRARY_DIRS ${RWS_LIBS_DIR})
#
# The include dirs
#
set(ROBWORKSTUDIO_INCLUDE_DIR ${RWS_ROOT}/src ${Boost_INCLUDE_DIR}
                              ${RWS_ROOT}/ext/qtpropertybrowser/src/
)

#
# The library dirs
#
set(ROBWORKSTUDIO_LIBRARY_DIRS ${Boost_LIBRARY_DIRS}
                               ${CMAKE_CURRENT_BINARY_DIR}/libs/${RWS_BUILD_TYPE}
)

#
# Setup the Library List here. We need to make sure the correct order is maintained which is crucial
# for some compilers.
#
set(ROBWORKSTUDIO_LIBRARIES_INTERNAL sdurws_robworkstudioapp sdurws_workcelleditor sdurws_luaeditor
                                     ${RWS_LUA} sdurws qtpropertybrowser
)
set(ROBWORKSTUDIO_PLUGIN_LIBRARIES
    sdurws_jog
    sdurws_log
    sdurws_playback
    sdurws_propertyview
    sdurws_treeview
    sdurws_planning
    sdurws_sensors
    sdurws_workcelleditorplugin
    sdurws_luapl
)

set(ROBWORKSTUDIO_LIBRARIES_EXTERNAL ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES}
                                     ${GLUT_glut_LIBRARY}
)

set(ROBWORKSTUDIO_LIBRARIES)
foreach(l ${ROBWORKSTUDIO_LIBRARIES_EXTERNAL})
    unset(tmp CACHE)
    find_library(
        tmp ${l}
        PATHS ${ROBWORKSTUDIO_LIBRARY_DIRS}
        NO_DEFAULT_PATH
    )
    if(tmp)
        list(APPEND ROBWORKSTUDIO_LIBRARIES ${tmp})
    else()
        list(APPEND ROBWORKSTUDIO_LIBRARIES ${l})
    endif()
endforeach(l)
set(ROBWORKSTUDIO_LIBRARIES ${ROBWORKSTUDIO_LIBRARIES_INTERNAL} ${ROBWORKSTUDIO_LIBRARIES})
