cmake_minimum_required (VERSION 3.5)
project (RobWorkSphinxDoc)

find_program(SPHINX_EXECUTABLE
             NAMES sphinx-build
             DOC "Sphinx Documentation Builder (sphinx-doc.org)")

find_program(DOXYGEN_EXECUTABLE doxygen
             HINTS ${DOXYGEN_PATH} $ENV{DOXYGEN_PATH}
             DOC "Doxygen API-doc Builder")

if(NOT SPHINX_EXECUTABLE)
  message(FATAL_ERROR "SPHINX_EXECUTABLE (sphinx-build) is not found!")
endif()
if(NOT DOXYGEN_EXECUTABLE)
  message(FATAL_ERROR "DOXYGEN_EXECUTABLE (doxygen) is not found!")
endif()

find_package(Java)	
if(NOT JAVA_FOUND)
  message(FATAL_ERROR "Java not found!")
endif()

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(RobWorkSim REQUIRED)

set(ROBWORK_VERSION ${RW_BUILD_WITH_VERSION})
configure_file(conf.py.in conf.py @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/../Doxyfile)

set(java_sources)
set(java_packages org.robwork)

foreach(RW_MODULE sdurw_core sdurw_common sdurw_math sdurw_kinematics sdurw_geometry sdurw_sensor sdurw_models sdurw_graspplanning sdurw_proximity sdurw sdurw_assembly sdurw_control sdurw_pathoptimization sdurw_pathplanners sdurw_proximitystrategies sdurw_task sdurw_simulation sdurw_opengl)
    add_custom_command(OUTPUT java_src_${RW_MODULE}_cmd
                       COMMAND ${CMAKE_COMMAND} -E echo "Unpacking jar file..."
                       COMMAND ${CMAKE_COMMAND} -E make_directory java_src
                       COMMAND ${CMAKE_COMMAND} -E chdir java_src ${Java_JAR_EXECUTABLE} xf ${RW_LIBS}/${RW_MODULE}_java-source.jar
                       COMMAND ${CMAKE_COMMAND} -E touch java_src_${RW_MODULE}_cmd
                       DEPENDS ${RW_LIBS}/${RW_MODULE}_java-source.jar
                       COMMENT "Unpack java source for ${RW_MODULE} module"
                       VERBATIM)
    list(APPEND java_sources java_src_${RW_MODULE}_cmd)
    set(java_packages ${java_packages} org.robwork.${RW_MODULE})
endforeach()

add_custom_command(OUTPUT java_src_sdurws_cmd
                   COMMAND ${CMAKE_COMMAND} -E echo "Unpacking jar file..."
                   COMMAND ${CMAKE_COMMAND} -E make_directory java_src
                   COMMAND ${CMAKE_COMMAND} -E chdir java_src ${Java_JAR_EXECUTABLE} xf ${RWS_LIBS}/sdurws_java-source.jar
                   COMMAND ${CMAKE_COMMAND} -E touch java_src_sdurws_cmd
                   DEPENDS ${RWS_LIBS}/sdurws_java-source.jar
                   COMMENT "Unpack java source for robworkstudio module"
                   VERBATIM)
list(APPEND java_sources java_src_sdurws_cmd)
set(java_packages ${java_packages} org.robwork.sdurws)

add_custom_command(OUTPUT java_src_sdurwsim_cmd
                   COMMAND ${CMAKE_COMMAND} -E echo "Unpacking jar file..."
                   COMMAND ${CMAKE_COMMAND} -E make_directory java_src
                   COMMAND ${CMAKE_COMMAND} -E chdir java_src ${Java_JAR_EXECUTABLE} xf ${RWSIM_LIBS}/sdurwsim_java-source.jar
                   COMMAND ${CMAKE_COMMAND} -E touch java_src_sdurwsim_cmd
                   DEPENDS ${RWSIM_LIBS}/sdurwsim_java-source.jar
                   COMMENT "Unpack java source for robworksim module"
                   VERBATIM)
list(APPEND java_sources java_src_sdurwsim_cmd)
set(java_packages ${java_packages} org.robwork.sdurwsim)

add_custom_target(java_src SOURCES ${java_sources})

add_custom_command(OUTPUT javadoc_cmd
                   COMMAND ${CMAKE_COMMAND} -E echo "Removing old Javadoc..."
                   COMMAND ${CMAKE_COMMAND} -E remove_directory javadoc
                   COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
                   COMMAND ${CMAKE_COMMAND} -E make_directory javadoc
                   COMMAND ${Java_JAVADOC_EXECUTABLE}
                           ${CLASSPATH}
                           -d javadoc
                           -windowtitle "RobWork Java API Documentation"
                           -public
                           -sourcepath java_src
                           ${java_packages}
                   COMMAND ${CMAKE_COMMAND} -E touch javadoc_cmd
                   DEPENDS ${java_sources}
                   VERBATIM)

add_custom_target(javadoc SOURCES javadoc_cmd)

add_custom_command(OUTPUT doxygen_cmd
                   COMMAND ${DOXYGEN_EXECUTABLE}
                   COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/doxygen_cmd
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
                   VERBATIM)

add_custom_target(doxygen SOURCES doxygen_cmd)


add_custom_command(OUTPUT doc_html
                   COMMAND ${SPHINX_EXECUTABLE}
                           -c ${CMAKE_CURRENT_BINARY_DIR}
                           -d ${CMAKE_CURRENT_BINARY_DIR}/doctrees
                           -b dirhtml
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_BINARY_DIR}/html
                           > ${CMAKE_CURRENT_BINARY_DIR}/html_output.log
                   COMMENT "sphinx-build html: see html_output.log"
                   VERBATIM)
add_custom_command(OUTPUT doc_latex
                   COMMAND ${SPHINX_EXECUTABLE}
                           -c ${CMAKE_CURRENT_BINARY_DIR}
                           -d ${CMAKE_CURRENT_BINARY_DIR}/doctrees
                           -b latex
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_BINARY_DIR}/latex
                           > ${CMAKE_CURRENT_BINARY_DIR}/latex_output.log
                   COMMENT "sphinx-build latex: see latex_output.log"
                   VERBATIM)
add_custom_command(OUTPUT doc_qt
                   COMMAND ${SPHINX_EXECUTABLE}
                           -c ${CMAKE_CURRENT_BINARY_DIR}
                           -d ${CMAKE_CURRENT_BINARY_DIR}/doctrees
                           -b qthelp
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_BINARY_DIR}/qthelp
                           > ${CMAKE_CURRENT_BINARY_DIR}/qthelp_output.log
                   COMMENT "sphinx-build qthelp: see qthelp_output.log"
                   VERBATIM)

add_custom_target(sphinx SOURCES doc_html)
add_custom_target(sphinx_latex SOURCES doc_latex)
add_custom_target(sphinx_qthelp SOURCES doc_qt)

add_custom_command(OUTPUT sphinxapidoc_cmd
	COMMAND ${CMAKE_COMMAND} -E echo "${CMAKE_CURRENT_SOURCE_DIR}/../../apidocs/html html/apidoc/cpp/doxygen"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../apidocs/html html/apidoc/cpp/doxygen
	COMMAND ${CMAKE_COMMAND} -E echo "javadoc html/apidoc/java/javadoc"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory javadoc html/apidoc/java/javadoc
                   COMMAND ${CMAKE_COMMAND} -E touch sphinxapidoc_cmd
                   DEPENDS javadoc_cmd doxygen_cmd
                   COMMENT "Copying apidoc into sphinx documentation.."
                   VERBATIM)

add_custom_target(sphinxapidoc SOURCES sphinxapidoc_cmd)
add_dependencies(sphinxapidoc doxygen)
add_dependencies(sphinxapidoc javadoc)
add_dependencies(sphinxapidoc sphinx)
