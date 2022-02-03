set(SUBSYS_NAME sdurws_luapl)
set(SUBSYS_DESC "A lua plugin")
set(SUBSYS_DEPS sdurws sdurws_luaeditor sdurws_lua_s sdurws_robworkstudioapp)

set(DEFAULT ${RWS_HAVE_LUA})
set(REASON)

set(build ${RWS_HAVE_LUA})
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    REASON ${REASON}
    ADD_DOC
)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles Lua.cpp)
    set(SRC_FILES_HPP Lua.hpp)

    # qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME}) Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles resources.qrc)
    endif()

    # Set extra linker flags for apple macos computers
    rws_add_plugin(${SUBSYS_NAME} ${RWS_DEFAULT_LIB_TYPE} EXPORT_SET RobWorkStudioluaTargets USING_SWIG ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/lua" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurws ${QT_LIBRARIES} RW::sdurw
        PRIVATE sdurws_luaeditor sdurws_lua_s
    )
    target_include_directories(
        ${SUBSYS_NAME} INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src>
                                 $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rws_plugin_load_details(${SUBSYS_NAME} 2 Lua false)
    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        #set(RWS_PLUGIN_LIBRARIES
        #    ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME}
        #    PARENT_SCOPE
        #)
    endif()

else()
    rws_clear_plugin_load_details(Lua)
endif()
