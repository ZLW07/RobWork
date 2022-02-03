set(SUBSYS_NAME sdurw_simulation)
set(SUBSYS_DESC "Simulated sensors and controllers")
set(SUBSYS_DEPS sdurw sdurw_control sdurw_core sdurw_common sdurw_kinematics sdurw_geometry sdurw_sensor sdurw_models)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    set(
        SRC_FILES
        ${RW_SENSORS_USER_FILES}
        FrameGrabber.cpp
        FrameGrabber25D.cpp
        GLFrameGrabber.cpp
        SimulatedCamera.cpp
        SimulatedKinect.cpp
        GLFrameGrabber25D.cpp
        KinematicSimulator.cpp
        SimulatedController.cpp
        SimulatedSensor.cpp
        Simulator.cpp
        SimulatedScanner25D.cpp
        SimulatedScanner2D.cpp
    )
    set(
        SRC_FILES_HPP
        FrameGrabber.hpp
        FrameGrabber25D.hpp
        GLFrameGrabber.hpp
        GLFrameGrabber25D.hpp
        KinematicSimulator.hpp
        SimulatedCamera.hpp
        SimulatedController.hpp
        SimulatedSensor.hpp
        Simulator.hpp
        SimulatedScanner25D.hpp
        SimulatedScanner2D.hpp
        SimulatedKinect.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/simulation" ${SRC_FILES_HPP})
    rw_add_include_dirs(${SUBSYS_NAME} "rwlibs/simulation" camera)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
