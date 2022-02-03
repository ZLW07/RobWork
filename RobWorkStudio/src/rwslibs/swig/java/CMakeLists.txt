set(SUBSYS_NAME sdurws_java)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from java.")
set(SUBSYS_DEPS sdurws sdurws_robworkstudioapp RW::sdurw_java)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

    find_package(Java)
    find_package(JNI)
    if(NOT (JAVA_FOUND AND JNI_FOUND))
        set(DEFAULT false)
        set(REASON "JAVA or JNI not found!")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseJava)
    include(UseSWIG)

    add_library(${SUBSYS_NAME} INTERFACE)

    include_directories(${RW_ROOT}/src)
    include_directories(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})

    # Put java files in different directory suitable for JAR generation later on

    rw_add_swig(
        sdurws java MODULE
        TARGET_NAME sdurws_jni
        INSTALL_DIR ${JAVA_INSTALL_DIR}
        SOURCES ../ScriptTypes.cpp
        CXX_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_sdurws/org/robwork/sdurws
        LANGUAGE_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_sdurws/org/robwork/sdurws
        SWIG_FLAGS "-package" "org.robwork.sdurws"
    )

    target_link_libraries(
        sdurws_jni
        sdurws
        sdurws_robworkstudioapp
        RW::sdurw
        RW::sdurw_core
        RW::sdurw_common
        RW::sdurw_math
        RW::sdurw_kinematics
        RW::sdurw_proximity
    )

    rw_add_java_clean_target(sdurws)

    # Compile java code and create JAR and Javadoc
    rw_add_java_lib(
        sdurws
        CLASSPATH ${RW_LIBS}/sdurw_core_java.jar
                  ${RW_LIBS}/sdurw_common_java.jar
                  ${RW_LIBS}/sdurw_math_java.jar
                  ${RW_LIBS}/sdurw_kinematics_java.jar
                  ${RW_LIBS}/sdurw_geometry_java.jar
                  ${RW_LIBS}/sdurw_sensor_java.jar
                  ${RW_LIBS}/sdurw_models_java.jar
                  ${RW_LIBS}/sdurw_proximity_java.jar
                  ${RW_LIBS}/sdurw_java.jar
        JAVADOC_LINK ${RW_LIBS}/javadoc/sdurw
                     ${RW_LIBS}/javadoc/sdurw_core
                     ${RW_LIBS}/javadoc/sdurw_common
                     ${RW_LIBS}/javadoc/sdurw_math
                     ${RW_LIBS}/javadoc/sdurw_kinematics
                     ${RW_LIBS}/javadoc/sdurw_geometry
                     ${RW_LIBS}/javadoc/sdurw_sensor
                     ${RW_LIBS}/javadoc/sdurw_models
                     ${RW_LIBS}/javadoc/sdurw_proximity
        WINDOW_TITLE "RobWorkStudio Java API Documentation"
    )
    set(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    if(TARGET sdurw_jni)
        add_dependencies(sdurws_jni sdurw_jni)
    endif()

    if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
        target_link_libraries(${SUBSYS_NAME} INTERFACE sdurws_jni)
    endif()

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}javaTargets)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    rw_swig_compile_target(java sdurws_jni ${java_NAME_sdurws}_java_doc ${java_NAME_sdurws}_libs)

endif()
