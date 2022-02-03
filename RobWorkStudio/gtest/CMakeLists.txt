########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWS_GTEST target)
	ADD_TEST(NAME ${target} COMMAND $<TARGET_FILE:${target}>)
	ADD_CUSTOM_TARGET(${target}_report-makedir
		COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${target}>/gtest_reports
		COMMENT "Creating directory gtest_reports if it does not exist."
	)
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND $<TARGET_FILE:${target}> --gtest_output=xml:$<TARGET_FILE_DIR:${target}>/gtest_reports/${target}.xml
		DEPENDS ${target} ${target}_report-makedir
	)
	SET(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
	IF(RW_BUILD_WITH_GTEST_SHARED_LIBS)
	  TARGET_COMPILE_DEFINITIONS(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
	  IF(MSVC)
		TARGET_COMPILE_OPTIONS(${target} PRIVATE /wd4251 /wd4275)
	  ENDIF()
	ENDIF()
ENDMACRO(ADD_RWS_GTEST)

########################################################################
# Configuration files for the testfiles path.
########################################################################

SET(ROBWORKSTUDIO_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated
)

FILE(GENERATE OUTPUT $<TARGET_FILE_DIR:sdurws_sdurws-gtest>/TestSuiteConfig.xml INPUT ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated)

########################################################################
# RobWorkStudio main function for initialization (link with this if needed).
########################################################################
set(CMAKE_AUTOMOC OFF)
set(RWSMAIN_TEST_SRC
  TestEnvironment.cpp
  test-main.cpp
)
add_library( sdurws-gtest-main STATIC ${RWSMAIN_TEST_SRC})
target_link_libraries( sdurws-gtest-main PUBLIC ${RW_BUILD_WITH_LIBRARIES_GTEST} RW::sdurw)

########################################################################
# RWS
########################################################################

SET(RWS_TEST_LIBRARIES
  sdurws-gtest-main
  sdurws
  sdurws_robworkstudioapp
  ${QT_LIBRARIES}
  ${RW_BUILD_WITH_LIBRARIES_GTEST}
  RW::sdurw
)

SET(RWS_TEST_SRC
  rws/RobWorkStudioTest.cpp
)
add_executable( sdurws_sdurws-gtest ${RWS_TEST_SRC})
target_link_libraries( sdurws_sdurws-gtest PRIVATE ${RWS_TEST_LIBRARIES})
target_include_directories(sdurws_sdurws-gtest PRIVATE  ${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})
ADD_RWS_GTEST(sdurws_sdurws-gtest)


########################################################################
# Test of simulated sensors
########################################################################

SET(SENSOR_TEST_LIBRARIES
    sdurws-gtest-main
    sdurws
    RW::sdurw_simulation
    RW::sdurw
)


SET(SENSOR_TEST_SRC
  rws/SimulatedSensorTest.cpp
)
add_executable( sdurws_sensor-gtest ${SENSOR_TEST_SRC})
target_link_libraries( sdurws_sensor-gtest PRIVATE ${SENSOR_TEST_LIBRARIES})
ADD_RWS_GTEST(sdurws_sensor-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(sdurws-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(sdurws-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
