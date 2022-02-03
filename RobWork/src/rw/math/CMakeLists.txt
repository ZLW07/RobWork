set(SUBSYS_NAME sdurw_math)
set(SUBSYS_DESC "This is a gathering of all RobWorks math")
set(SUBSYS_DEPS sdurw_core sdurw_common)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        CameraMatrix.cpp
        EAA.cpp
        EigenDecomposition.cpp
        InertiaMatrix.cpp
        Jacobian.cpp
        Line2D.cpp
        Line2DPolar.cpp
        LinearAlgebra.cpp
        Math.cpp
        Metric.cpp
        MetricFactory.cpp
        MetricUtil.cpp
        PerspectiveTransform2D.cpp
        Polynomial.cpp
        PolynomialND.cpp
        PolynomialSolver.cpp
        Pose2D.cpp
        Pose6D.cpp
        ProjectionMatrix.cpp
        Q.cpp
        Quaternion.cpp
        RPY.cpp
        Random.cpp
        Rotation2D.cpp
        Rotation3D.cpp
        Rotation3DVector.cpp
        Statistics.cpp
        Transform2D.cpp
        Transform3D.cpp
        Transform3DVector.cpp
        Vector.cpp
        Vector2D.cpp
        Vector3D.cpp
        VectorND.cpp
        VelocityScrew6D.cpp
        Wrench6D.cpp
    )

    set(FILES_HPP
        CameraMatrix.hpp
        Constants.hpp
        EAA.hpp
        EigenDecomposition.hpp
        Function.hpp
        InertiaMatrix.hpp
        Jacobian.hpp
        Line2D.hpp
        Line2DPolar.hpp
        LinearAlgebra.hpp
        Math.hpp
        Metric.hpp
        MetricFactory.hpp
        MetricUtil.hpp
        PerspectiveTransform2D.hpp
        Polynomial.hpp
        PolynomialND.hpp
        PolynomialSolver.hpp
        Pose2D.hpp
        Pose6D.hpp
        ProjectionMatrix.hpp
        Q.hpp
        Quaternion.hpp  
        Random.hpp
        Rotation2D.hpp
        Rotation3D.hpp
        Rotation3DVector.hpp
        RPY.hpp
        Statistics.hpp
        Transform2D.hpp
        Transform3D.hpp
        Transform3DVector.hpp
        Vector.hpp
        Vector2D.hpp
        Vector3D.hpp
        VectorND.hpp
        VelocityScrew6D.hpp
        Wrench6D.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/math" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})

    rw_include_eigen(${SUBSYS_NAME})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../math.hpp)

    if(RW_HAVE_OMP)
        target_link_libraries(${SUBSYS_NAME} PUBLIC ${OpenMP_CXX_LIBRARIES})
    endif()
endif()
