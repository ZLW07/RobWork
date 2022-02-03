set(SUBSYS_NAME sdurw_sandbox)
set(SUBSYS_DESC "Stuff not yet implemented in robwork")
set(SUBSYS_DEPS sdurw)

set(build FALSE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF DEPENDS ${SUBSYS_DEPS})

if(build)

    set(SRC_FILES
        ${RW_SANDBOX_USER_FILES}
        # loaders/ColladaLoader.cpp loaders/ColladaSaver.cpp loaders/RWXMLFile.cpp
        # MovingAverage.cpp
        invkin/BasicGPMM.cpp
        invkin/IKGPMMSolver.cpp
        ConvertUtil.cpp
        calibration/EncoderDecentralization.cpp
        # ClarkHullND.cpp
        csg/CSGModel.cpp
        # algorithms/ConstraintModel.cpp
        # algorithms/ConstraintGenerator.cpp algorithms/PointConstraint.cpp
        # algorithms/LineConstraint.cpp
        #
        # algorithms/BoxConstraint.cpp algorithms/PlaneConstraint.cpp
        # algorithms/FixtureConstraint.cpp algorithms/ConstraintSandbox.cpp
        algorithms/ConstraintModel.cpp
        # algorithms/StablePoseConstraint.cpp
        optimization/Optimizer.cpp
        optimization/StopCondition.cpp
        optimization/OptimizerFactory.cpp
        optimization/DownhillOptimizer.cpp
        optimization/LineSearch.cpp
        optimization/GoldenSectionLineSearch.cpp
        optimization/DirectionSetOptimizer.cpp
        optimization/TaxiCabOptimizer.cpp
        optimization/PowellOptimizer.cpp
        optimization/GradientOptimizer.cpp
        optimization/GradientDescentOptimizer.cpp
        optimization/BFGSOptimizer.cpp
        optimization/SimulatedAnnealingOptimizer.cpp
    )

    set(SRC_FILES_HPP
        # ClarkHullND.hpp
        # MovingAverage.hpp
        ConvertUtil.hpp
        calibration/EncoderDecentralization.hpp
        csg/CSGModel.hpp
        # algorithms/ConstraintGenerator.hpp algorithms/PointConstraint.hpp
        # algorithms/LineConstraint.hpp algorithms/StablePoseConstraint.hpp
        # algorithms/BoxConstraint.hpp algorithms/PlaneConstraint.hpp
        # algorithms/FixtureConstraint.hpp algorithms/ConstraintSandbox.hpp
        algorithms/ConstraintModel.hpp
        # algorithms/StablePoseConstraint.hpp
        optimization/Optimizer.hpp
        optimization/StopCondition.hpp
        optimization/OptimizerFactory.hpp
        optimization/LineSearch.hpp
        optimization/GoldenSectionLineSearch.hpp
        optimization/DownhillOptimizer.hpp
        optimization/DirectionSetOptimizer.hpp
        optimization/TaxiCabOptimizer.hpp
        optimization/PowellOptimizer.hpp
        optimization/GradientOptimizer.hpp
        optimization/GradientDescentOptimizer.hpp
        optimization/BFGSOptimizer.hpp
        optimization/SimulatedAnnealingOptimizer.hpp
    )

    add_library(sdurw_sandbox ${SRC_FILES} ${SRC_FILES_HPP})
    target_link_libraries(sdurw_sandbox sdurw_csgjs sdurw)
    install(TARGETS sdurw_sandbox DESTINATION ${LIB_INSTALL_DIR})
    install(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/sandbox)

endif()
