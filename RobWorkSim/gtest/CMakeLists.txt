# ##################################################################################################
# Standard Macro
# ##################################################################################################

macro(ADD_RWSIM_GTEST target)
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
    if(RW_BUILD_WITH_GTEST_SHARED_LIBS)
        target_compile_definitions(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
        if(MSVC)
            target_compile_options(${target} PRIVATE /wd4251 /wd4275)
        endif()
    endif()
endmacro(ADD_RWSIM_GTEST)

# ##################################################################################################
# RobWork main function for initialization (link with this if needed).
# ##################################################################################################

add_library(sdurwsim-gtest-main STATIC TestEnvironment.cpp test-main.cpp)
target_link_libraries(sdurwsim-gtest-main ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw sdurwsim_test)

# ##################################################################################################
# Common
# ##################################################################################################
if(TARGET sdurwsim)
    set(COMMON_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw)

    set(COMMON_TEST_SRC common/CommonTest.cpp)

    add_executable(sdurwsim_common-gtest ${COMMON_TEST_SRC})
    target_link_libraries(sdurwsim_common-gtest ${COMMON_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_common-gtest)
endif()
# ##################################################################################################
# Dynamics
# ##################################################################################################
if(TARGET sdurwsim)
    set(DYNAMICS_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw)

    set(DYNAMICS_TEST_SRC dynamics/ConstraintTest.cpp)

    add_executable(sdurwsim_dynamics-gtest ${DYNAMICS_TEST_SRC})
    target_link_libraries(sdurwsim_dynamics-gtest ${DYNAMICS_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_dynamics-gtest)
endif()
# ##################################################################################################
# Ode
# ##################################################################################################

if(TARGET sdurwsim_ode)

    set(ODE_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim sdurwsim_ode ${RW_BUILD_WITH_LIBRARIES_GTEST}
                           RW::sdurw
    )

    set(ODE_TEST_SRC ode/ODEControlTest.cpp ode/ODESimpleTest)

    add_executable(sdurwsim_ode-gtest ${ODE_TEST_SRC})
    target_link_libraries(sdurwsim_ode-gtest ${ODE_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_ode-gtest)

endif()

# ##################################################################################################
# Performance
# ##################################################################################################

if(TARGET sdurwsim_ode)

    set(PERFORMANCE_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim sdurwsim_ode
                                   ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw
    )

    set(PERFORMANCE_TEST_SRC performance/BenchMarkSimple.cpp performance/BenchMarkStacking.cpp)

    add_executable(sdurwsim_performance-gtest ${PERFORMANCE_TEST_SRC})
    target_link_libraries(sdurwsim_performance-gtest ${PERFORMANCE_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_performance-gtest)

endif()

# ##################################################################################################
# Sensors
# ##################################################################################################
if(TARGET sdurwsim_ode)

    set(SENSORS_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim sdurwsim_ode ${RW_BUILD_WITH_LIBRARIES_GTEST}
                               RW::sdurw
    )

    set(SENSORS_TEST_SRC sensors/FTSensorTest.cpp sensors/TactileArraySensorTest.cpp)

    add_executable(sdurwsim_sensors-gtest ${SENSORS_TEST_SRC})
    target_link_libraries(sdurwsim_sensors-gtest ${SENSORS_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_sensors-gtest)

endif()

# ##################################################################################################
# Simulator
# ##################################################################################################

if(TARGET sdurwsim)
    set(SIMULATOR_TEST_LIBRARIES sdurwsim_test sdurwsim ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw)
    set(SIMULATOR_TEST_SRC simulator/PhysicsEngineTest.cpp)
    add_executable(sdurwsim_simulator-gtest ${SIMULATOR_TEST_SRC})
    target_link_libraries(sdurwsim_simulator-gtest ${SIMULATOR_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_simulator-gtest)
endif()

# ##################################################################################################
# Tasksimulation
# ##################################################################################################

if(TARGET sdurwsim_ode)
    set(TASKSIMULATION_TEST_LIBRARIES sdurwsim-gtest-main sdurwsim sdurwsim_ode
                                      ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw RW::sdurw_task
    )
    set(TASKSIMULATION_TEST_SRC tasksimulation/GraspTaskSimulation.cpp)
    add_executable(sdurwsim_tasksimulation-gtest ${TASKSIMULATION_TEST_SRC})
    target_link_libraries(sdurwsim_tasksimulation-gtest ${TASKSIMULATION_TEST_LIBRARIES})
    add_rwsim_gtest(sdurwsim_tasksimulation-gtest)
endif()

# ##################################################################################################
# TestFiles
# ##################################################################################################

# setup the test file configuration
set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
if(MSVC)
    set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE})
endif()

set(ROBWORKSIM_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)
get_filename_component(ROBWORKSIM_XMLSCHEMAS_DIR ../xml-schemas ABSOLUTE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
               ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml
)

# ##################################################################################################
# Target for generation of all detailed reports
# ##################################################################################################

add_custom_target(
    sdurwsim-gtest_reports
    DEPENDS ${REPORT_TARGETS}
    COMMENT "Running Google Tests to generate detailed reports."
)

# ##################################################################################################
# Do not build these as part of an ordinary build
# ##################################################################################################

set_target_properties(
    sdurwsim-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1
                                                        EXCLUDE_FROM_DEFAULT_BUILD 1
)
