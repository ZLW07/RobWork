set(SUBSYS_NAME sdurw_graspplanning)
set(SUBSYS_DESC "A collection of graspplanning algorithms!")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_geometry sdurw_sensor)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        ApproachMeasure3D.cpp
        CMDistCCPMeasure3D.cpp
        CompositeContactFilter.cpp
        CompositeGraspFilter.cpp
        ContactDistThresFilter.cpp
        ContactValidateFilter.cpp
        Contour2DInfoMap.cpp
        CurvatureThresFilter.cpp
        DiceContactG3D.cpp
        GWSMeasure3D.cpp
        Grasp2D.cpp
        Grasp3D.cpp
        GraspQualityMeasure3D.cpp
        GraspTable.cpp
        GraspValidateFilter.cpp
        PlaneClearanceFilter.cpp
        SemiForceClosureFilter.cpp
        WrenchMeasure3D.cpp
    )

    set(FILES_HPP
        ApproachMeasure3D.hpp
        CMDistCCPMeasure3D.hpp
        CompositeContactFilter.hpp
        CompositeGraspFilter.hpp
        ContactDistThresFilter.hpp
        ContactValidateFilter.hpp
        Contour2DInfoMap.hpp
        CurvatureThresFilter.hpp
        DiceContactG3D.hpp
        Grasp2D.hpp
        Grasp3D.hpp
        GraspQualityMeasure3D.hpp
        GraspTable.hpp
        GraspValidateFilter.hpp
        PlaneClearanceFilter.hpp
        SemiForceClosureFilter.hpp
        WrenchMeasure3D.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/graspplanning" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ../graspplanning.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
