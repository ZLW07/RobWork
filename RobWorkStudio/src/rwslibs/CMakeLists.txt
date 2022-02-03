# add_definitions(-DQT_PLUGIN)

if(DEFINED MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D QT_NO_DEBUG")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -D QT_NO_DEBUG")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -D QT_NO_DEBUG")
else()
    add_definitions(-DQT_NO_DEBUG)
endif()

set(RWS_DEFAULT_LIB_TYPE MODULE)
if(RWS_USE_STATIC_LINK_PLUGINS)
    set(RWS_DEFAULT_LIB_TYPE STATIC)
endif()

# The plugins:
add_subdirectory(atask)
add_subdirectory(gtask)
add_subdirectory(jog)
# add_subdirectory(pointer)
add_subdirectory(log)
add_subdirectory(playback)
add_subdirectory(propertyview)
add_subdirectory(treeview)
add_subdirectory(planning)
add_subdirectory(pythoneditor)
add_subdirectory(sensors)
add_subdirectory(workcelleditor)
add_subdirectory(workcelleditorplugin)
# depends on all above
add_subdirectory(rwstudioapp)
# depends on rwstudio
add_subdirectory(swig)
# These need to be after swig
add_subdirectory(luaeditor)
add_subdirectory(lua)

# Propagate libraries to parent directory
set(RWS_PLUGIN_LIBRARIES
    ${RWS_PLUGIN_LIBRARIES}
    PARENT_SCOPE
)
set(RWS_COMPONENT_LIBRARIES
    ${RWS_COMPONENT_LIBRARIES}
    PARENT_SCOPE
)
