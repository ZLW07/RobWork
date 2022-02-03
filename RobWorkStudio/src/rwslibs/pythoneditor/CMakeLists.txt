set(SUBSYS_NAME sdurws_pythoneditor)
set(SUBSYS_DESC "A python editor")
set(SUBSYS_DEPS sdurws RW::sdurw_python RW::sdurw)

if(${PYTHONLIBS_FOUND} AND ${PYTHONINTERP_FOUND} AND ${RWS_USE_QCODEEDITOR})
    set(DEFAULT ON)
    set(REASON)
else()
    set(DEFAULT OFF)
    set(REASON "Either Pyton or QCodeEditor is not installed")
endif()



# Verify that sdurws_python will be build
if(
    ${DEFAULT}
    AND (DEFINED BUILD_sdurws_robworkstudioapp AND NOT ${BUILD_sdurws_robworkstudioapp})
    AND (DEFINED BUILD_sdurws_python AND NOT ${BUILD_sdurws_python})
)
    set(DEFAULT OFF)
    set(REASON "Unmet dependencies sdurws_python or sdurws_robworkstudioapp")
endif()

set(build true)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    REASON ${REASON}
    ADD_DOC
)

if(build)
    # Standard cpp files to compile:
    set(SrcFiles Editor.cpp)
    set(SRC_FILES_HPP Editor.hpp)

    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles resources_pyeditor.qrc ../../rws/rwstudio_resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles resources_pyeditor.qrc ../../rws/rwstudio_resources.qrc)
    endif()

    # Set extra linker flags for apple macos computers
    rws_add_plugin(${SUBSYS_NAME} ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${RccSrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwslibs/pythoneditor" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurws ${QT_LIBRARIES} RW::sdurw
        PRIVATE ${PYTHON_LIBRARIES} QCodeEditor
    )
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        PRIVATE ${PYTHON_INCLUDE_DIRS}
    )

    rws_plugin_load_details(${SUBSYS_NAME} 2 PythonEditor false)
    if("${RWS_DEFAULT_LIB_TYPE}" STREQUAL "STATIC")
        set(RWS_PLUGIN_LIBRARIES ${RWS_PLUGIN_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
    endif()
endif()
