set(SUBSYS_NAME SimulatorLogViewer)
set(SUBSYS_DESC "Stand-alone application for visualization of internal data from a Physics Engine.")
set(SUBSYS_DEPS sdurws sdurw)

set(build TRUE)
rw_subsys_option(
    build
    ${SUBSYS_NAME}
    ${SUBSYS_DESC}
    ON
)
rw_add_doc(${SUBSYS_NAME})

if(build)
    # Standard cpp files to compile:
    set(SrcFiles SimulatorLogViewer.cpp)
    set(SRC_FILES_HPP SimulatorLogViewer.hpp)

    set_source_files_properties(
        ${SrcFiles}
        PROPERTIES
        OBJECT_DEPENDS
        "${UIS_OUT_H}"
    )

    set(MocSrcFiles ${SRC_FILES_HPP})
    set(RccSrcFiles resources.qrc)

    add_executable(SimulatorLogViewer ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_FILES})
    target_link_libraries(
        SimulatorLogViewer
        PRIVATE
        sdurwsim_gui
    )
    # Need to add the current binary dir to the include directory because the generated source files are placed here
    target_include_directories(SimulatorLogViewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

endif()
