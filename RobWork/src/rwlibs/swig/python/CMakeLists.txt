set(SUBSYS_NAME sdurw_python)
set(SUBSYS_DESC "Interface for accessing RobWork from python.")
set(SUBSYS_DEPS)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)

set(DEFAULT True)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()
    if(NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND))
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
    else()
        set(REASON "Configured by default")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseSWIG)

    add_library(${SUBSYS_NAME} INTERFACE)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    include_directories(${RW_ROOT}/src)
    include_directories(${PYTHON_INCLUDE_DIRS})

    if(RW_USE_NUMPY)
        include_directories(${NUMPY_INCLUDE_DIR})
    endif()

    set(SWIG_FLAGS "-includeall")
    if((PYTHON_VERSION_MAJOR VERSION_GREATER 3) OR (PYTHON_VERSION_MAJOR VERSION_EQUAL 3))
        set(SWIG_FLAGS "${SWIG_FLAGS};-py3")
    endif()

    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set(SWIG_FLAGS "${SWIG_FLAGS};-doxygen")
    endif()

    configure_file(setup.py.in ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)
    configure_file(RobWork.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWork-${ROBWORK_VERSION}.egg-info
    )

    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWork-${ROBWORK_VERSION}.egg-info
            DESTINATION ${PYTHON_INSTALL_DIR}
    )

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    set(SWIG_OUT_ORIG ${CMAKE_SWIG_OUTDIR})
    set(python_targets)

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
        sdurw_pathoptimization
        sdurw_pathplanners
        sdurw_proximitystrategies
        sdurw_simulation
        sdurw_opengl
    )
        if(TARGET ${RW_MODULE})
            set(SOURCE)
            if("${RW_MODULE}" STREQUAL "sdurw")
                set(SOURCE ../ScriptTypes.cpp)
            endif()
            set(TARGET_NAME ${RW_MODULE}${LIB_SUFIX})
            rw_add_swig(
                ${RW_MODULE} python MODULE
                TARGET_NAME ${TARGET_NAME}
                INSTALL_DIR ${PYTHON_INSTALL_DIR}/${RW_MODULE}
                SOURCES ${SOURCE}
                LANGUAGE_FILE_DIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}
                                  BINARY_OUTPUT_DIR
                                  "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}"
                SWIG_FLAGS ${SWIG_FLAGS}
            )

            if(NOT POLICY CMP0078)
                set(TARGET_NAME ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME})
            endif()

            # Set build destination
            target_compile_definitions(${TARGET_NAME} PUBLIC RW_USE_NUMPY)
            add_dependencies(${SUBSYS_NAME} ${TARGET_NAME})
            set(python_targets ${python_targets} ${TARGET_NAME})

            if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
                find_program(pydoc pydoc3)
                if(NOT pydoc)
                    find_program(pydoc pydoc)
                endif()
                if(pydoc)
                    add_custom_command(
                        TARGET ${TARGET_NAME}
                        POST_BUILD
                        COMMAND ${pydoc} -w
                                ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}/${RW_MODULE}.py
                        WORKING_DIRECTORY ${SWIG_OUT_ORIG}
                        COMMENT "Creating pydoc..."
                    )
                endif()
            endif()

            if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
                target_link_libraries(${SUBSYS_NAME} INTERFACE ${TARGET_NAME})
            endif()

            install(
                FILES ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}/${RW_MODULE}.py
                DESTINATION "${PYTHON_INSTALL_DIR}/${RW_MODULE}"
                COMPONENT swig
            )
            install(
                FILES ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}/__init__.py
                DESTINATION "${PYTHON_INSTALL_DIR}/${RW_MODULE}"
                COMPONENT swig
            )

            get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})
            target_link_libraries(
                ${python_NAME_${RW_MODULE}} PUBLIC ${RW_MODULE} ${depends} ${PYTHON_LIBRARIES}
            )

            set(EXTRA_MODULES)

            foreach(module ${depends})
                set(EXTRA_MODULES "${EXTRA_MODULES}from ${module} import *\n")
            endforeach()
            configure_file(__init__.py.in
                           ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}/__init__.py
            )

        endif()
    endforeach()

    rw_swig_compile_target(python ${python_targets})

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}pythonTargets)

endif()
