set(SUBSYS_NAME sdurws_atask)
set(SUBSYS_DESC "A plugin for visualization of assembly tasks and results.")
set(SUBSYS_DEPS sdurws)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # if we want to use ui files add them here
    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H_ATaskVisPlugin ATaskVisPlugin.ui)
        qt_wrap_ui(UIS_OUT_H_StrategyLibraryDialog StrategyLibraryDialog.ui)
        qt_wrap_ui(UIS_OUT_H_TaskSetupDialog TaskSetupDialog.ui)
    else()
        qt5_wrap_ui(UIS_OUT_H_ATaskVisPlugin ATaskVisPlugin.ui)
        qt5_wrap_ui(UIS_OUT_H_StrategyLibraryDialog StrategyLibraryDialog.ui)
        qt5_wrap_ui(UIS_OUT_H_TaskSetupDialog TaskSetupDialog.ui)
    endif()
    set(UIS_OUT_H ${UIS_OUT_H_ATaskVisPlugin} ${UIS_OUT_H_StrategyLibraryDialog} ${UIS_OUT_H_TaskSetupDialog})

    # Standard cpp files to compile:
    set(SrcFiles ATaskVisPlugin.cpp StrategyLibraryDialog.cpp TaskSetupDialog.cpp)
    set(SRC_FILES_HPP ATaskVisPlugin.hpp StrategyLibraryDialog.hpp TaskSetupDialog.hpp)

    set_source_files_properties(ATaskVisPlugin.cpp PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_ATaskVisPlugin}")
    set_source_files_properties(
        StrategyLibraryDialog.cpp
        PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_StrategyLibraryDialog}"
    )
    set_source_files_properties(TaskSetupDialog.cpp PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H_TaskSetupDialog}")

    #qt5_wrap_cpp(MocSrcFiles ${SRC_FILES_HPP} TARGET ${SUBSYS_NAME})
    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(RccSrcFiles atask_resources.qrc)
    else()
        qt5_add_resources(RccSrcFiles atask_resources.qrc)
    endif()

    rws_add_plugin(
        ${SUBSYS_NAME}
        MODULE
        ${SrcFiles}
        ${RccSrcFiles}
        ${UIS_OUT_H}
    )
    rw_add_includes(${SUBSYS_NAME} "rwslibs/atask" ${SRC_FILES_HPP} ${UIS_OUT_H})
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    target_include_directories(
        ${SUBSYS_NAME}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
    )
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} ${QT_LIBRARIES} PRIVATE RW::sdurw_assembly RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
endif()
