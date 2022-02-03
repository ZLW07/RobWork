set(SUBSYS_NAME sdurw_control)
set(SUBSYS_DESC "A collection of general controllers and interfaces for RobWork")
set(SUBSYS_DEPS sdurw sdurw_core sdurw_math sdurw_models)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES Controller.cpp JointController.cpp SyncVelocityRamp.cpp)

    set(SRC_FILES_HPP Controller.hpp JointController.hpp SyncVelocityRamp.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/control" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()
