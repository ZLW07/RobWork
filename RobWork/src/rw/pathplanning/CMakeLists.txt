set(SUBSYS_NAME sdurw_pathplanning)
set(SUBSYS_DESC "The pathplanning RobWork library!")
set(SUBSYS_DEPS
    sdurw_core
    sdurw_math
    sdurw_kinematics
    sdurw_proximity
    sdurw_models
    sdurw_trajectory
    sdurw_invkin
)
set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)
    set(FILES_CPP
        PlannerConstraint.cpp
        QEdgeConstraint.cpp
        QEdgeConstraintIncremental.cpp
        PathPlanner.cpp
        QToQPlanner.cpp
        QToTPlanner.cpp
        QToQSamplerPlanner.cpp
        PlannerUtil.cpp
        StopCriteria.cpp
        #QConstraint.cpp
        #QNormalizer.cpp
        QIKSampler.cpp
        QSampler.cpp
        #StateConstraint.cpp
        PathAnalyzer.cpp
    )

    set(FILES_HPP
        PlannerConstraint.hpp
        QEdgeConstraint.hpp
        QEdgeConstraintIncremental.hpp
        PathPlanner.hpp
        QToQPlanner.hpp
        QToTPlanner.hpp
        QToQSamplerPlanner.hpp
        PlannerUtil.hpp
        StopCriteria.hpp
        QConstraint.hpp
        QNormalizer.hpp
        QIKSampler.hpp
        QSampler.hpp
        StateConstraint.hpp
        PathAnalyzer.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/pathplanning" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ../pathplanning.hpp)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()
