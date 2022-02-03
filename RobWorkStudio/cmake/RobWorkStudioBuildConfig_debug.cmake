
# ######################################################################################################################
# Version of RobWork

set(RWS_BUILD_WITH_VERSION 22.2.3)
set(RWS_BUILD_WITH_VERSION_MAJOR 22)
set(RWS_BUILD_WITH_VERSION_MINOR 2)
set(RWS_BUILD_WITH_VERSION_PATCH 3)

# ######################################################################################################################
# dependencies
set(RWS_BUILD_WITH_LUA False)

set(RWS_BUILD_WITH_PYTHON true)
set(RWS_BUILD_WITH_PYTHON_MAJOR_VERSION 3)
set(RWS_BUILD_WITH_PYTHONLIBS_MAJOR_VERSION )

set(RWS_BUILD_WITH_FREEGLUT FALSE)

set(RWS_BUILD_WITH_BOOST_LIBRARY_DIR "")

# ######################################################################################################################
# flags and definitions
set(RWS_BUILD_WITH_RWS_ROOT "/home/zw/CLionProjects/RobWork/RobWorkStudio")
set(RWS_BUILD_WITH_RW_ROOT "/home/zw/CLionProjects/RobWork/RobWork/")
set(RWS_BUILD_WITH_CXX_FLAGS " -Wall -Wno-strict-aliasing -Wno-unused-function -fPIC -fopenmp")
set(RWS_BUILD_WITH_DEFINITIONS ;-DQT_DEBUG)
set(RWS_BUILD_WITH_LINKER_FLAGS )
set(RWS_BUILD_WITH_BUILD_DIR "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkStudio")

# ######################################################################################################################
# and libraries
set(RWS_BUILD_WITH_LIBRARIES "sdurws_robworkstudioapp;sdurws_workcelleditor;sdurws_luaeditor;sdurws;qtpropertybrowser")
set(RWS_BUILD_WITH_LIB_DEPEND "Qt5::Core;Qt5::Gui;Qt5::Widgets;Qt5::OpenGL;/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/usr/lib/x86_64-linux-gnu/libglut.so")
set(RWS_BUILD_WITH_PLUGIN_LIBRARIES "sdurws_jog;sdurws_log;sdurws_playback;sdurws_propertyview;sdurws_treeview;sdurws_planning;sdurws_sensors;sdurws_workcelleditorplugin;sdurws_luapl")

set(RWS_BUILD_WITH_LIBRARY_DIRS "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkStudio/libs/debug")
set(RWS_BUILD_WITH_LIBRARY_SUBDIRS "x86_64-linux-gnu" "x86_64-linux-gnu/RobWork" "x86_64-linux-gnu/RobWork/rwsplugins" "RobWork" "RobWork/rwsplugins" "x86_64-linux-gnu/RobWork/static" "RobWork/static")
set(RWS_BUILD_WITH_INCLUDE_DIR "/home/zw/CLionProjects/RobWork/RobWorkStudio/src;/home/zw/CLionProjects/RobWork/RobWorkStudio/ext/qtpropertybrowser/src/")
