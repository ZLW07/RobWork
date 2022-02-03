set(SUBSYS_NAME sdurws_sensors)
set(SUBSYS_DESC "A sensor view plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES SensorsPlugin.ui)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()

    # Standard cpp files to compile:
    set(SrcFiles Sensors.cpp SensorView.cpp)
    set(SRC_FILES_HPP Sensors.hpp SensorView.hpp)

    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        ${RWS_DEFAULT_LIB_TYPE}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/sensors" ${SRC_FILES_HPP})
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} ${QT_LIBRARIES} PRIVATE RW::sdurw_simulation PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rws_plugin_load_details(${SUBSYS_NAME} 2 Sensors false)
    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
