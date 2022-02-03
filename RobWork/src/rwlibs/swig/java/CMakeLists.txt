set(SUBSYS_NAME sdurw_java)
set(SUBSYS_DESC "Interface for accessing RobWork from java.")
set(SUBSYS_DEPS)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)
set(DEFAULT True)
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
    else()
        set(REASON "Configured by default.")
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

    set(build_doc TRUE)
    if(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(build_doc FALSE)
    endif()
    rw_subsys_option(build sdurw_java_doc "build documentation for the java implementation"
                     ${build_doc} REASON "set by default"
    )

    add_library(${SUBSYS_NAME} INTERFACE)

    include_directories(${RW_ROOT}/src)
    include_directories(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
    set(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    set(java_targets)

    foreach(
        RW_MODULE
        sdurw_core
        sdurw_common
        sdurw_math
        sdurw_kinematics
        sdurw_plugin
        sdurw_geometry
        sdurw_sensor
        sdurw_models
        sdurw_proximity
        sdurw_graphics
        sdurw_trajectory
        sdurw_invkin
        sdurw_pathplanning
        sdurw_graspplanning
        sdurw_loaders
        sdurw
        sdurw_algorithms
        sdurw_task
        sdurw_assembly
        sdurw_control
        sdurw_pathplanners
        sdurw_proximitystrategies
        sdurw_pathoptimization
        sdurw_simulation
        sdurw_opengl
    )
        if(TARGET ${RW_MODULE})
            set(TARGET_NAME ${RW_MODULE}_jni)
            set(SOURCE)
            if("${RW_MODULE}" STREQUAL "sdurw")
                set(SOURCE ../ScriptTypes.cpp)
            endif()

            rw_add_swig(
                ${RW_MODULE} java MODULE
                TARGET_NAME ${TARGET_NAME}
                INSTALL_DIR ${JAVA_INSTALL_DIR}
                SOURCES ${SOURCE}
                CXX_FILE_DIR
                    ${CMAKE_CURRENT_BINARY_DIR}/java_src_${RW_MODULE}/org/robwork/${RW_MODULE}
                LANGUAGE_FILE_DIR
                    ${CMAKE_CURRENT_BINARY_DIR}/java_src_${RW_MODULE}/org/robwork/${RW_MODULE}
                SWIG_FLAGS "-package" "org.robwork.${RW_MODULE}"
            )

            set(java_targets ${java_targets} ${TARGET_NAME} ${java_NAME_${RW_MODULE}}_java_doc
                             ${java_NAME_${RW_MODULE}}_libs
            )

            # Force removal of previous Java compilation and source when interface file changes This
            # is required as types may be removed or change name (in this case previous java classes
            # would interfere with current compilation).
            rw_add_java_clean_target(${RW_MODULE})

            get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})

            set(class_path_depend "${depends}")
            set(link_path_depend "${depends}")
            if(CMAKE_VERSION VERSION_GREATER 3.12.0)
                list(TRANSFORM class_path_depend PREPEND "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/")
                list(TRANSFORM class_path_depend APPEND "_java.jar")
                list(TRANSFORM link_path_depend PREPEND "../")
            else()
                set(cpd)
                set(lpd)
                foreach(dep ${depends})
                    set(cpd ${cpd} "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${dep}_java.jar")
                    set(lpd ${lpd} "../${dep}")
                endforeach()
                set(class_path_depend "${cpd}")
                set(link_path_depend "${lpd}")
            endif()

            foreach(dep ${depends})
                add_dependencies(${TARGET_NAME} ${java_NAME_${dep}} ${java_NAME_${dep}}_java_doc
                ${java_NAME_${dep}}_libs)
            endforeach()

            if("${RW_MODULE}" STREQUAL "sdurw_core")

                set(LOADERRW_FILE java_src_${RW_MODULE}/org/robwork/LoaderRW.java)
                set(LOADERRW_PKG org.robwork)
                set(LOADERRW_CP_CMD copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRW.java
                                    java_src_${RW_MODULE}/org/robwork/LoaderRW.java
                )
            else()
                set(LINK_JAVADOC ${link_path_depend})
                set(LOADERRW_FILE)
                set(LOADERRW_PKG)
                set(LOADERRW_CP_CMD echo " - Nothing to copy")
            endif()
            # Compile java code and create JAR and Javadoc

            rw_add_java_lib(
                ${RW_MODULE}
                CLASSPATH ${class_path_depend}
                JAVADOC_LINK ${link_path_depend}
                WINDOW_TITLE "RobWork Java API Documentation"
                BUILD_DOC ${build_doc}
            )

            if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
                target_link_libraries(${SUBSYS_NAME} INTERFACE ${RW_MODULE}_jni)
            endif()

            target_link_libraries(${RW_MODULE}_jni PUBLIC ${RW_MODULE} ${depends})
        endif()
    endforeach()

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}javaTargets)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    RW_SWIG_COMPILE_TARGET(java ${java_targets})


endif()
