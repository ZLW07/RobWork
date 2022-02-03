set(SUBSYS_NAME sdurws_workcelleditorplugin)
set(SUBSYS_DESC "Workcell editor plugin")
set(SUBSYS_DEPS sdurws sdurws_workcelleditor)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    # Standard cpp files to compile:
    set(SrcFiles WorkcellEditorPlugin.cpp)
    set(SRC_FILES_HPP WorkcellEditorPlugin.hpp)

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles resources.qrc)
    endif()

    # Set extra linker flags for apple macos computers
    rws_add_plugin(
        ${SUBSYS_NAME}
        ${RWS_DEFAULT_LIB_TYPE}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/workcelleditorplugin" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} PRIVATE ${QT_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()

endif()
