set(SUBSYS_NAME sdurws_gtask)
set(SUBSYS_DESC "A sensor view plugin")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    set(UIS_FILES GTaskVisPlugin.ui)
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()
    # Standard cpp files to compile:
    set(SrcFiles GTaskVisPlugin.cpp)
    set(SRC_FILES_HPP GTaskVisPlugin.hpp)

    # set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles gtask_resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles gtask_resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        MODULE
        ${UIS_OUT_H}
        ${SrcFiles}
        ${MocSrcFiles}
        ${RccSrcFiles}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/gtask" ${SRC_FILES_HPP} ${UIS_FILES})
    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(
        ${SUBSYS_NAME}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
    )
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC ${SUBSYS_DEPS} ${QT_LIBRARIES}
        PRIVATE ${OPENGL_LIBRARIES} RW::sdurw_task RW::sdurw
    )
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
endif()
