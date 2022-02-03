set(SUBSYS_NAME sdurw_trajectory)
set(SUBSYS_DESC "The trajectory RobWork library!")
set(SUBSYS_DEPS sdurw_core sdurw_math sdurw_models)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)

    set(FILES_CPP
        Blend.cpp
        CircularInterpolator.cpp
        CubicSplineFactory.cpp
        CubicSplineInterpolator.cpp
        Interpolator.cpp
        InterpolatorUtil.cpp
        LinearInterpolator.cpp
        LloydHaywardBlend.cpp
        ParabolicBlend.cpp
        Path.cpp
        Timed.cpp
        TimedUtil.cpp
        Trajectory.cpp
        TrajectoryFactory.cpp
        TrajectoryIterator.cpp
        InterpolatorTrajectory.cpp
        BlendedTrajectory.cpp
        TimeMetricUtil.cpp
        RampInterpolator.cpp
    )

    set(FILES_HPP
        Blend.hpp
        CircularInterpolator.hpp
        RampInterpolator.hpp
        CubicSplineFactory.hpp
        CubicSplineInterpolator.hpp
        Interpolator.hpp
        InterpolatorUtil.hpp
        LinearInterpolator.hpp
        LloydHaywardBlend.hpp
        ParabolicBlend.hpp
        Path.hpp
        Timed.hpp
        TimedUtil.hpp
        Trajectory.hpp
        TrajectoryFactory.hpp
        TrajectoryIterator.hpp
        InterpolatorTrajectory.hpp
        BlendedTrajectory.hpp
        TimeMetricUtil.hpp
        TrajectorySequence.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/trajectory" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ../trajectory.hpp)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})

    rw_include_eigen(${SUBSYS_NAME})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()
