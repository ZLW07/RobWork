set(SUBSYS_NAME sdurw)
set(SUBSYS_DESC "A collection of all RobWork primary librarys!")
set(SUBSYS_DEPS
    sdurw_core
    sdurw_common
    sdurw_math
    sdurw_kinematics
    sdurw_plugin
    sdurw_geometry
    sdurw_sensor
    sdurw_models
    sdurw_proximity
    sdurw_graphics
    sdurw_trajectory
    sdurw_invkin
    sdurw_pathplanning
    sdurw_graspplanning
    sdurw_loaders
)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)

    # Add the RobWork files
    set(FILES_HPP RobWork.hpp rw.hpp math_fwd.hpp)

    add_library(sdurw INTERFACE)
    add_library(RW::sdurw ALIAS sdurw)
    rw_add_includes(${SUBSYS_NAME} "rw" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} INTERFACE ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME} INTERFACE $<BUILD_INTERFACE:${RW_ROOT}/src>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>)

    install(
        TARGETS sdurw
        EXPORT ${PROJECT_PREFIX}Targets
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT sdurw
        LIBRARY DESTINATION ${lib_dir} COMPONENT sdurw
        ARCHIVE DESTINATION ${lib_dir} COMPONENT sdurw
    )

endif()
