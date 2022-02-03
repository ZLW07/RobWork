set(SUBSYS_NAME sdurw_algorithms)
set(SUBSYS_DESC "A collection of varius algorithms!")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(
        SRC_FILES
        ${RW_ALGORITHMS_USER_FILES}
        ./kdtree/KDTree.cpp
        ./kdtree/KDTreeQ.cpp
        ./qpcontroller/IKQPSolver.cpp
        ./qpcontroller/QPController.cpp
        ./qpcontroller/QPSolver.cpp
        ./xqpcontroller/XQPController.cpp
        ./xqpcontroller/NullSpaceProjection.cpp
        ./gpm/BasicGPM.cpp
        BFGS.cpp
        MovingAverage.cpp
        # StablePlanePose.cpp
        RANSACModel.cpp
        PointModel.cpp
        LineModel.cpp
        StructuredLineModel.cpp
        PlaneModel.cpp
        StablePose1DModel.cpp
        StablePose0DModel.cpp
        PointPairsRegistration.cpp
    )

    set(
        SRC_FILES_HPP
        ./kdtree/KDTree.hpp
        ./kdtree/KDTreeQ.hpp
        ./qpcontroller/IKQPSolver.hpp
        ./qpcontroller/QPController.hpp
        ./qpcontroller/QPSolver.hpp
        ./xqpcontroller/XQPController.hpp
        ./xqpcontroller/NullSpaceProjection.hpp
        ./gpm/BasicGPM.hpp
        BFGS.hpp
        MovingAverage.hpp
        # StablePlanePose.hpp
        RANSACModel.hpp
        PointModel.hpp
        LineModel.hpp
        StructuredLineModel.hpp
        PlaneModel.hpp
        StablePose1DModel.hpp
        StablePose0DModel.hpp
        PointPairsRegistration.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/algorithms" MovingAverage.hpp StablePlanePose.hpp)
    rw_add_include_dirs(
        ${SUBSYS_NAME}
        "rwlibs/algorithms"
        kdtree
        qpcontroller
        xqpcontroller
        gpm
    )
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
