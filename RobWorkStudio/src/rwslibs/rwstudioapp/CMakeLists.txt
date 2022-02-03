set(SUBSYS_NAME sdurws_robworkstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
set(SUBSYS_DEPS sdurws RW::sdurw)

if(RWS_USE_STATIC_LINK_PLUGINS)
    set(SUBSYS_DEPS_TEST
        sdurws_jog
        sdurws_log
        sdurws_planning
        sdurws_playback
        sdurws_pythoneditor
        sdurws_sensors
        sdurws_treeview
        sdurws_propertyview
        sdurws_workcelleditorplugin
    )
    if(SWIG_DEFAULT_COMPILE)
        set(SUBSYS_DEPS_TEST ${SUBSYS_DEPS_TEST} sdurws_luaeditor sdurws_luapl)
    endif()
    rw_is_targets(test TARGETS ${SUBSYS_DEPS_TEST} TARGETS_OUT static_plugins)
    list(APPEND SUBSYS_DEPS ${static_plugins})
endif()

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles RobWorkStudioApp.cpp)
    set(SRC_FILES_HPP RobWorkStudioApp.hpp)

    rws_add_component(${SUBSYS_NAME} ${SrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/rwstudioapp" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurws ${QT_LIBRARIES}
        PRIVATE ${SUBSYS_DEPS} ${GLUT_glut_LIBRARY}
    )
    target_include_directories(
        ${SUBSYS_NAME} INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src>
                                 $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    if(RWS_USE_STATIC_LINK_PLUGINS)

        if(TARGET sdurws_jog)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_JOG)
        endif()
        if(TARGET sdurws_log)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_LOG)
        endif()
        if(TARGET sdurws_planning)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PLANNING)
        endif()
        if(TARGET sdurws_playback)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PLAYBACK)
        endif()
        if(TARGET sdurws_propertyview)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PROPERTYVIEW)
        endif()
        if(TARGET sdurws_sensors)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_SENSORS)
        endif()
        if(TARGET sdurws_treeview)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_TREEVIEW)
        endif()
        if(TARGET sdurws_workcelleditorplugin)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_WORKCELLEDITOR)
        endif()
        if(TARGET sdurws_luapl AND SWIG_DEFAULT_COMPILE)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_LUAPL)
        endif()
        if(TARGET sdurws_pythoneditor)
            target_compile_definitions(${SUBSYS_NAME} PRIVATE RWS_HAVE_PLUGIN_PYTHONEDITOR)
        endif()
    endif()

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
