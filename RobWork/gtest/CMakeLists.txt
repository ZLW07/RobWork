include_directories(${GTEST_INCLUDE_DIRS})

# ##################################################################################################
# Standard Macro
# ##################################################################################################

macro(ADD_RW_GTEST target)
    add_test(NAME ${target} COMMAND $<TARGET_FILE:${target}>)
    add_custom_target(
        ${target}_report-makedir
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${target}>/gtest_reports
        COMMENT "Creating directory gtest_reports if it does not exist."
    )
    add_custom_target(
        ${target}_report
        COMMAND $<TARGET_FILE:${target}>
                --gtest_output=xml:$<TARGET_FILE_DIR:${target}>/gtest_reports/${target}.xml
        DEPENDS ${target} ${target}_report-makedir
    )
    set(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
    if(GTEST_SHARED_LIBS)
        target_compile_definitions(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
        if(MSVC)
            target_compile_options(${target} PRIVATE /wd4251 /wd4275)
        endif()
    endif()
endmacro()

# ##################################################################################################
# RobWork main function for initialization (link with this if needed).
# ##################################################################################################

set(RWMAIN_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_core)

set(RWMAIN_TEST_SRC TestEnvironment.cpp test-main.cpp)
add_library(sdurw-gtest-main STATIC ${RWMAIN_TEST_SRC})
target_link_libraries(sdurw-gtest-main ${RWMAIN_TEST_LIBRARIES})

# ##################################################################################################
# Algorithms
# ##################################################################################################

if(TARGET sdurw_algorithms)
    set(ALGORITHMS_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw sdurw_algorithms)

    set(ALGORITHMS_TEST_SRC algorithms/RANSACTest.cpp)
    add_executable(sdurw_algorithms-gtest ${ALGORITHMS_TEST_SRC})
    target_link_libraries(sdurw_algorithms-gtest ${ALGORITHMS_TEST_LIBRARIES})
    add_rw_gtest(sdurw_algorithms-gtest)
endif()

# ##################################################################################################
# Calibration
# ##################################################################################################

if(TARGET sdurw_calibration)
    set(CALIBRATION_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw sdurw_calibration)

    set(CALIBRATION_TEST_SRC calibration/CalibrationTest.cpp)
    add_executable(sdurw_calibration-gtest ${CALIBRATION_TEST_SRC})
    target_link_libraries(sdurw_calibration-gtest ${CALIBRATION_TEST_LIBRARIES})
    add_rw_gtest(sdurw_calibration-gtest)
endif()

# ##################################################################################################
# Core
# ##################################################################################################

if(TARGET sdurw_core)
    set(CORE_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw_core)

    set(CORE_TEST_SRC
        core/AnyPtrTest.cpp
        core/EventTest.cpp
        core/PluginTest.cpp
        core/PropertyMapTest.cpp
        core/PropertyTest.cpp
        core/PropertyTypeCoreTest.cpp
        core/PropertyValueTest.cpp
        core/PtrTest.cpp
        core/PtrBoostTest.cpp
        core/StringUtilTest.cpp
    )

    add_executable(sdurw_core-gtest ${CORE_TEST_SRC})
    target_link_libraries(sdurw_core-gtest ${CORE_TEST_LIBRARIES})
    add_rw_gtest(sdurw_core-gtest)

    # Create dummy plugins for testing
    add_library(test_plugin.rwplugin MODULE core/TestPlugin.cpp)
    target_link_libraries(test_plugin.rwplugin sdurw_core)
    set_target_properties(
        test_plugin.rwplugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    )

    # Create XML file for lazy-loading of Test plugin
    file(
        GENERATE
        OUTPUT "$<TARGET_FILE_DIR:test_plugin.rwplugin>/test_plugin.rwplugin.xml"
        INPUT ${CMAKE_CURRENT_SOURCE_DIR}/core/test_plugin.rwplugin.xml.in
    )
endif()

# ##################################################################################################
# Common
# ##################################################################################################
if(TARGET sdurw_common AND TARGET sdurw_math)
    set(COMMON_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_common sdurw_math)

    set(COMMON_TEST_SRC
        common/ThreadTest.cpp
        common/StringUtilTest.cpp
        common/SerializationTest.cpp
        common/PropertyTest.cpp
        common/IteratorTest.cpp
        common/PairMapTest.cpp
        common/ProgramOptionTest.cpp
        common/TimerTest.cpp
        common/LogTestSuite.cpp
    )

    add_executable(sdurw_common-gtest ${COMMON_TEST_SRC})
    target_link_libraries(sdurw_common-gtest ${COMMON_TEST_LIBRARIES})
    add_rw_gtest(sdurw_common-gtest)
endif()

# ##################################################################################################
# Geometry
# ##################################################################################################

if(TARGET sdurw)
    set(GEOMETRY_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw)

    set(GEOMETRY_TEST_SRC
        geometry/DelaunayTest.cpp
        geometry/BSphereTest.cpp
        geometry/HyperSphereTest.cpp
        geometry/ImplicitTorusTest.cpp
        geometry/IndexedTriMeshTest.cpp
        geometry/IntersectUtilTest.cpp
        geometry/PlaneTest.cpp
        geometry/PolygonTest.cpp
        geometry/QHullTest.cpp
        geometry/QuadraticCurveTest.cpp
        geometry/TriangulateTest.cpp
        geometry/GeometryTest.cpp
        geometry/GeometryUtilTest.cpp
        geometry/TriangleUtilTest.cpp
    )
    add_executable(sdurw_geometry-gtest ${GEOMETRY_TEST_SRC})
    target_link_libraries(sdurw_geometry-gtest ${GEOMETRY_TEST_LIBRARIES})
    add_rw_gtest(sdurw_geometry-gtest)
endif()

# ##################################################################################################
# Graphics
# ##################################################################################################

if(TARGET sdurw)
    set(GRAPHICS_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw)

    set(GRAPHICS_TEST_SRC graphics/SceneGraphTest.cpp graphics/WorkCellSceneTest.cpp
                          graphics/GraphicsTest.cpp
    )
    add_executable(sdurw_graphics-gtest ${GRAPHICS_TEST_SRC})
    target_link_libraries(sdurw_graphics-gtest ${GRAPHICS_TEST_LIBRARIES})
    add_rw_gtest(sdurw_graphics-gtest)
endif()

# ##################################################################################################
# Inverse Kinematics
# ##################################################################################################

if(TARGET sdurw)
    set(INVKIN_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw)

    set(INVKIN_TEST_SRC
        invkin/ClosedFormIKSolverKukaIIWATest.cpp invkin/ParallelIKSolverTest.cpp
        invkin/JacobianIKSolverTest.cpp invkin/ClosedFormIKSolverURTest.cpp invkin/InvKinTest.cpp
    )
    add_executable(sdurw_invkin-gtest ${INVKIN_TEST_SRC})
    target_link_libraries(sdurw_invkin-gtest ${INVKIN_TEST_LIBRARIES})
    add_rw_gtest(sdurw_invkin-gtest)
endif()

# ##################################################################################################
# Kinematics
# ##################################################################################################

if(TARGET sdurw)
    set(KINEMATICS_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

    set(KINEMATICS_TEST_SRC kinematics/StaticFrameGroupsTest.cpp kinematics/KinematicsTest.cpp)
    add_executable(sdurw_kinematics-gtest ${KINEMATICS_TEST_SRC})
    target_link_libraries(sdurw_kinematics-gtest ${KINEMATICS_TEST_LIBRARIES})
    add_rw_gtest(sdurw_kinematics-gtest)
endif()

# ##################################################################################################
# Loaders
# ##################################################################################################

if(TARGET sdurw)
    set(LOADERS_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw sdurw_proximitystrategies)

    set(LOADERS_TEST_SRC
        loaders/DOMProximitySetupSaver.cpp loaders/DOMPropertyMap.cpp loaders/ImageLoaderTest.cpp
        loaders/PathLoaderCSVTest.cpp loaders/TrajectoryLoaderTest.cpp
    )
    add_executable(sdurw_loaders-gtest ${LOADERS_TEST_SRC})
    target_link_libraries(sdurw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
    add_rw_gtest(sdurw_loaders-gtest)
endif()

# ##################################################################################################
# Math
# ##################################################################################################

if(TARGET sdurw)
    set(MATH_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

    set(MATH_TEST_SRC
        math/EAATest.cpp
        math/MetricFactoryTest.cpp
        math/PolynomialTest.cpp
        math/SerializationTest.cpp
        math/StatisticsTest.cpp
        math/QuaternionTest.cpp
        math/Vector3DTest.cpp
        math/VectorNDTest.cpp
        math/LinearAlgebraTest.cpp
        math/MathSerializationTest.cpp
        math/Pose6DTest.cpp
        math/PropertyTypeMathTest.cpp
        math/Rotation3DTest.cpp
        math/RPYTest.cpp
        math/Transform3DTest.cpp
        math/UtilTest.cpp
        math/Vector2DTest.cpp
        math/VelocityScrew6DTest.cpp
        math/Wrench6DTest.cpp
    )

    add_executable(sdurw_math-gtest ${MATH_TEST_SRC})
    target_link_libraries(sdurw_math-gtest ${MATH_TEST_LIBRARIES})
    add_rw_gtest(sdurw_math-gtest)
endif()

# ##################################################################################################
# Mathematica
# ##################################################################################################

if(TARGET sdurw_mathematica)
    set(MATHEMATICA_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw sdurw_mathematica)

    set(MATHEMATICA_TEST_SRC mathematica/MathematicaTest.cpp)
    add_executable(sdurw_mathematica-gtest ${MATHEMATICA_TEST_SRC})
    target_link_libraries(sdurw_mathematica-gtest ${MATHEMATICA_TEST_LIBRARIES})
    add_rw_gtest(sdurw_mathematica-gtest)
endif()

# ##################################################################################################
# Models
# ##################################################################################################

if(TARGET sdurw)
    set(MODELS_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw)

    set(MODELS_TEST_SRC
        models/JointTest.cpp models/ParallelDeviceTest.cpp models/ParallelLegTest.cpp
        models/WorkCellTest.cpp models/PrismaticJointTest.cpp models/SerialDeviceTest.cpp
    )
    add_executable(sdurw_models-gtest ${MODELS_TEST_SRC})
    target_link_libraries(sdurw_models-gtest ${MODELS_TEST_LIBRARIES})
    add_rw_gtest(sdurw_models-gtest)
endif()

# ##################################################################################################
# Opengl
# ##################################################################################################

if(TARGET sdurw_opengl)
    set(OPENGL_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw sdurw_opengl)

    set(OPENGL_TEST_SRC opengl/DrawableTest.cpp)
    add_executable(sdurw_opengl-gtest ${OPENGL_TEST_SRC})
    target_link_libraries(sdurw_opengl-gtest ${OPENGL_TEST_LIBRARIES})
    add_rw_gtest(sdurw_opengl-gtest)
endif()

# ##################################################################################################
# Pathoptimization
# ##################################################################################################

if(TARGET sdurw)
    set(PATHOPTIMIZATION_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_pathoptimization sdurw)

    set(PATHOPTIMIZATION_TEST_SRC pathoptimization/ClearanceOptimizerTest.cpp
                                  pathoptimization/PathLengthOptimizerTest.cpp
    )
    add_executable(sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_SRC})
    target_link_libraries(sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_LIBRARIES})
    add_rw_gtest(sdurw_pathoptimization-gtest)
endif()

# ##################################################################################################
# Pathplanning
# ##################################################################################################

if(TARGET sdurw_pathplanners)
    set(PATHPLANNING_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw_pathplanners sdurw
                                    sdurw_math sdurw_proximitystrategies
    )

    set(PATHPLANNING_TEST_SRC pathplanning/PathPlanningTest.cpp)

    add_executable(sdurw_pathplanning-gtest ${PATHPLANNING_TEST_SRC})
    target_link_libraries(sdurw_pathplanning-gtest ${PATHPLANNING_TEST_LIBRARIES})
    add_rw_gtest(sdurw_pathplanning-gtest)
endif()
# ##################################################################################################
# Proximity
# ##################################################################################################

if(TARGET sdurw_proximitystrategies)
    set(PROXIMITY_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw
                                 sdurw_proximitystrategies
    )

    set(PROXIMITY_TEST_SRC
        proximity/BVTreeTest.cpp
        proximity/CollisionStrategyTest.cpp
        proximity/CollisionToleranceStrategyTest.cpp
        proximity/DistanceMultiStrategyTest.cpp
        proximity/DistanceStrategyTest.cpp
        proximity/ProximityStrategyTest.cpp
        proximity/DistanceCalculatorTest.cpp
        proximity/ProximityModelTest.cpp
        proximity/ProximityCalculatorTest.cpp
        proximity/CollisionTest.cpp
        proximity/DistanceTest.cpp
        proximity/collisionStrategy.cpp
        proximity/distanceStrategy.cpp
        proximity/ProximitySetupRuleTest.cpp
    )
    add_executable(sdurw_proximity-gtest ${PROXIMITY_TEST_SRC})
    target_link_libraries(sdurw_proximity-gtest ${PROXIMITY_TEST_LIBRARIES})
    add_rw_gtest(sdurw_proximity-gtest)
endif()

# ##################################################################################################
# Sensor
# ##################################################################################################

if(TARGET sdurw)
    set(SENSOR_TEST_SRC sensor/TactileArrayTest.cpp)
    add_executable(sdurw_sensor-gtest ${SENSOR_TEST_SRC})
    target_link_libraries(sdurw_sensor-gtest ${GTEST_BOTH_LIBRARIES} sdurw)
    add_rw_gtest(sdurw_sensor-gtest)
endif()

# ##################################################################################################
# Task
# ##################################################################################################

if(TARGET sdurw_task)
    set(TASK_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw_task sdurw)

    set(TASK_TEST_SRC task/TaskLoaderTest.cpp task/TaskSaverTest.cpp task/GraspTaskTest.cpp)
    add_executable(sdurw_task-gtest ${TASK_TEST_SRC})
    target_link_libraries(sdurw_task-gtest ${TASK_TEST_LIBRARIES})
    add_rw_gtest(sdurw_task-gtest)
endif()

# ##################################################################################################
# Trajectory
# ##################################################################################################

if(TARGET sdurw AND TARGET sdurw_control)
    set(TRAJECTORY_TEST_LIBRARIES sdurw-gtest-main ${GTEST_BOTH_LIBRARIES} sdurw sdurw_control)
    set(TRAJECTORY_TEST_SRC
        trajectory/PathTest.cpp
        trajectory/ParabolicBlendTest.cpp
        trajectory/SyncVelocityRampTest.cpp
        trajectory/trajectory.cpp
        trajectory/TrajectoryFactoryTest.cpp
        trajectory/CubicSplineTest.cpp
        trajectory/LinearInterpolatorTest.cpp
        trajectory/PropertyTypeTrajectoryTest.cpp
    )
    add_executable(sdurw_trajectory-gtest ${TRAJECTORY_TEST_SRC})
    target_link_libraries(sdurw_trajectory-gtest ${QHULL_LIBRARIES} ${TRAJECTORY_TEST_LIBRARIES})
    add_rw_gtest(sdurw_trajectory-gtest)
endif()

# ##################################################################################################
# TestFiles
# ##################################################################################################

# setup the test file configuration
set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
if(MSVC)
    set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE})
endif()

set(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)
get_filename_component(ROBWORK_XMLSCHEMAS_DIR ../xml-schemas ABSOLUTE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
               ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml
)

# ##################################################################################################
# Target for generation of all detailed reports
# ##################################################################################################

add_custom_target(
    sdurw-gtest_reports
    DEPENDS ${REPORT_TARGETS}
    COMMENT "Running Google Tests to generate detailed reports."
)

# ##################################################################################################
# Do not build these as part of an ordinary build
# ##################################################################################################

set_target_properties(
    sdurw-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD
                                                     1
)
