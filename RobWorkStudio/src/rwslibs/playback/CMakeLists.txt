set(SUBSYS_NAME sdurws_playback)
set(SUBSYS_DESC "A playback plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles PlayBack.cpp Player.cpp StateDraw.cpp PlayBackSettings.cpp)
    set(SRC_FILES_HPP PlayBack.hpp Player.hpp PlayBackSettings.hpp)

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles playback_resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles playback_resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        ${RWS_DEFAULT_LIB_TYPE}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/playback" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} ${QT_LIBRARIES} RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rws_plugin_load_details(${SUBSYS_NAME} 8 PlayBack false)
    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
