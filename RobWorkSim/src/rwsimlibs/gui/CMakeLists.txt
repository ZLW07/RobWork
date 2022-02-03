set(SUBSYS_NAME sdurwsim_gui)
set(SUBSYS_DESC "RobWorkSim gui")
set(SUBSYS_DEPS sdurwsim )

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    # ##############################################################################################
    # first compile the general stuff into a static gui lib
    set(
        FILES_UI
        RestingPoseDialog.ui
        GraspSelectionDialog.ui
        GraspRestingPoseDialog.ui
        SupportPoseAnalyserDialog.ui
        CreateEngineDialog.ui
        SimCfgDialog.ui
        SDHPlugin.ui
        TactileSensorDialog.ui
        # GraspTableGeneratorPlugin.ui
        log/SimulatorLogWidget.ui
        log/BodyMotionWidget.ui
        log/CollisionResultWidget.ui
        log/DistanceResultWidget.ui
        log/ConstraintWidget.ui
        log/ForceTorqueWidget.ui
        log/ContactSetWidget.ui
        log/ContactVelocitiesWidget.ui
        log/EquationSystemWidget.ui
        log/LogValuesWidget.ui
        log/SimulatorStatisticsWidget.ui
    )

    set(
        SrcFiles_HPP
        ContactTableWidget.hpp
        JointControlDialog.hpp
        JogGroup.hpp
        RestingPoseDialog.hpp
        GraspSelectionDialog.hpp
        GraspRestingPoseDialog.hpp
        SupportPoseAnalyserDialog.hpp
        GLViewRW.hpp
        CreateEngineDialog.hpp
        SimCfgDialog.hpp
        BodyControllerWidget.hpp
        TactileSensorDialog.hpp
    )

    set(
        SrcFiles_Log_HPP
        log/SimulatorLogWidget.hpp
        log/SimulatorLogModel.hpp
        log/SimulatorLogEntryWidget.hpp
        log/BodyMotionWidget.hpp
        log/CollisionResultWidget.hpp
        log/DistanceMultiResultWidget.hpp
        log/DistanceResultWidget.hpp
        log/ConstraintWidget.hpp
        log/ForceTorqueWidget.hpp
        log/ContactSetWidget.hpp
        log/ContactVelocitiesWidget.hpp
        log/EquationSystemWidget.hpp
        log/LogValuesWidget.hpp
        log/LogMessageWidget.hpp
        log/SimulatorLogWidget.hpp
        log/SimulatorStatisticsWidget.hpp
    )

    set(
        SrcFiles
        ContactTableWidget.cpp
        JointControlDialog.cpp
        JogGroup.cpp
        BodyControllerWidget.cpp
        RestingPoseDialog.cpp
        GraspSelectionDialog.cpp
        GraspRestingPoseDialog.cpp
        SupportPoseAnalyserDialog.cpp
        GLViewRW.cpp
        CreateEngineDialog.cpp
        SimCfgDialog.cpp
        # TactilePadItem.cpp
        TactileSensorDialog.cpp
        log/SimulatorLogModel.cpp
        log/SimulatorLogEntryWidget.cpp
        log/SimulatorLogWidget.cpp
        log/BodyMotionWidget.cpp
        log/CollisionResultWidget.cpp
        log/DistanceMultiResultWidget.cpp
        log/DistanceResultWidget.cpp
        log/ConstraintWidget.cpp
        log/ForceTorqueWidget.cpp
        log/ContactSetWidget.cpp
        log/ContactVelocitiesWidget.cpp
        log/EquationSystemWidget.cpp
        log/LogValuesWidget.cpp
        log/LogMessageWidget.cpp
        log/SimulatorLogWidget.cpp
        log/SimulatorStatisticsWidget.cpp
        log/MathematicaPlotWidget.cpp
    )
    set(ODE_LIB)
    if(RWSIM_HAVE_ODE)
        set(SrcFiles_HPP ${SrcFiles_HPP} ODESimCfgDialog.hpp)
        set(SrcFiles ${SrcFiles} ODESimCfgDialog.cpp)
        set(ODE_LIB sdurwsim_ode)
    endif()
    set(MocSrcFiles ${SrcFiles_HPP} ${SrcFiles_Log_HPP})
    set(RccSrcFiles resources.qrc)

    rw_add_library(${SUBSYS_NAME} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/gui" ${SrcFiles_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/gui/log" ${SrcFiles_Log_HPP})

    target_link_libraries(
        sdurwsim_gui
        PUBLIC
            sdurwsim
            RWS::sdurws
            ${QT_LIBRARIES}
        PRIVATE ${ODE_LIB}
    )

    if(TARGET RW::sdurw_mathematica)
        target_link_libraries(sdurwsim_gui PRIVATE RW::sdurw_mathematica)
    endif()
    # to be able to include the generated ui header files
    target_include_directories(
        sdurwsim_gui

        INTERFACE $<BUILD_INTERFACE:${RWSIM_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        PRIVATE  ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
