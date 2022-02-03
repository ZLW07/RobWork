set(SUBSYS_NAME sdurw_proximity)
set(SUBSYS_DESC "Everything to do with calculating the proximity between objects")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_geometry sdurw_models sdurw_kinematics)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        BasicFilterStrategy.cpp
        CollisionDetector.cpp
        CollisionSetup.cpp
        CollisionStrategy.cpp
        CollisionToleranceStrategy.cpp
        DistanceCalculator.cpp
        DistanceMultiStrategy.cpp
        DistanceStrategy.cpp
        ProximityCache.cpp
        ProximityCalculator.cpp
        ProximityData.cpp
        ProximityFilter.cpp
        ProximityFilterStrategy.cpp
        ProximityModel.cpp
        ProximitySetup.cpp
        ProximitySetupRule.cpp
        ProximityStrategy.cpp
        ProximityStrategyData.cpp
        Raycaster.cpp
        rwstrategy/BVTree.cpp
        rwstrategy/BVTreeCollider.cpp
        rwstrategy/BVTreeColliderFactory.cpp
        rwstrategy/BVTreeFactory.cpp
        rwstrategy/BVTreeToleranceCollider.cpp
        rwstrategy/BinaryBVTree.cpp
        rwstrategy/BinaryIdxBVTree.cpp
        rwstrategy/OBVTreeDFSCollider.cpp
        rwstrategy/ProximityStrategyRW.cpp
    )

    set(FILES_HPP
        BasicFilterStrategy.hpp
        CollisionDetector.hpp
        CollisionSetup.hpp
        CollisionStrategy.hpp
        CollisionToleranceStrategy.hpp
        DistanceCalculator.hpp
        DistanceMultiStrategy.hpp
        DistanceStrategy.hpp
        ProximityCache.hpp
        ProximityCalculator.hpp
        ProximityData.hpp
        ProximityFilter.hpp
        ProximityFilterStrategy.hpp
        ProximityModel.hpp
        ProximitySetup.hpp
        ProximitySetupRule.hpp
        ProximityStrategy.hpp
        ProximityStrategyData.hpp
        Raycaster.hpp
    )
    set(FILES_HPP_rwstrategy
        rwstrategy/BVTree.hpp
        rwstrategy/BVTreeCollider.hpp
        rwstrategy/BVTreeColliderFactory.hpp
        rwstrategy/BVTreeFactory.hpp
        rwstrategy/BVTreeToleranceCollider.hpp
        rwstrategy/BinaryBVTree.hpp
        rwstrategy/BinaryIdxBVTree.hpp
        rwstrategy/OBVTreeDFSCollider.hpp
        rwstrategy/ProximityStrategyRW.hpp
    )

    if(RW_HAVE_PQP)
        set(FILES_CPP ${FILES_CPP} rwstrategy/TreeDistanceCalc.cpp)
        set(FILES_HPP_rwstrategy ${FILES_HPP_rwstrategy} rwstrategy/TreeDistanceCalc.hpp)
    endif()

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP} ${FILES_HPP_rwstrategy})
    rw_add_includes(${SUBSYS_NAME} "rw/proximity" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/proximity/rwstrategy" ${FILES_HPP_rwstrategy})
    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../proximity.hpp)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()
