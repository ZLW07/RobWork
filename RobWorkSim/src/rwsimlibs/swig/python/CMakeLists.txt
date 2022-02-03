set(SUBSYS_NAME sdurwsim_python)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from python.")
set(SUBSYS_DEPS sdurwsim RW::sdurw_python RWS::sdurws_python)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()
    if(NOT (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND))
        set(DEFAULT false)
        set(REASON "PYTHONLIBS AND PYTHONINTERP not found!")
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
    include_directories(${PYTHON_INCLUDE_DIRS})
    if(RW_USE_NUMPY)
        include_directories(${NUMPY_INCLUDE_DIR})
    endif()

    if(RW_USE_NUMPY)
    include_directories(${NUMPY_INCLUDE_DIR})
    endif()

    add_library(${SUBSYS_NAME} INTERFACE)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    include_directories(${RWSIM_ROOT}/src ${RW_ROOT}/src)

    set(RWSIM_MODULE sdurwsim)
    configure_file(RobWorkSim.egg-info.in
                   ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkSim-${ROBWORKSIM_VERSION}.egg-info)

    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobWorkSim-${ROBWORKSTUDIO_VERSION}.egg-info
        DESTINATION ${PYTHON_INSTALL_DIR}
    )

    if(POLICY CMP0078)
        set(LIB_SUFIX _py)
    else()
        set(LIB_SUFIX)
    endif()

    # ############ SWIG COMPILE #################
    set(TARGET_NAME ${RWSIM_MODULE}${LIB_SUFIX})
    rw_add_swig(
        ${RWSIM_MODULE} python MODULE
        TARGET_NAME ${TARGET_NAME}
        INSTALL_DIR ${PYTHON_INSTALL_DIR}/${RWSIM_MODULE}
        LANGUAGE_FILE_DIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}
        BINARY_OUTPUT_DIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}
        SOURCES ../ScriptTypes.cpp
        SWIG_FLAGS ${SWIG_FLAGS}
    )

    if(NOT POLICY CMP0078)
        set(TARGET_NAME ${SWIG_MODULE_sdurwsim${LIB_SUFIX}_REAL_NAME})
    endif()

    set(RW_USE_NUMPY ${RW_BUILD_WITH_NUMPY})
    target_compile_definitions(${TARGET_NAME} PRIVATE RW_USE_NUMPY)


    # ############# Documentation ##############
    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        find_program(pydoc pydoc3)
        if(NOT pydoc)
            find_program(pydoc pydoc)
        endif()
        if(pydoc)
            add_custom_command(
                TARGET ${TARGET_NAME} POST_BUILD
                COMMAND
                    ${CMAKE_COMMAND}
                    -E
                    env
                    PYTHONPATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../../../RobWork/libs/${RW_BUILD_TYPE}
                    ${pydoc}
                    -w
                    ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}/${RWSIM_MODULE}.py
                WORKING_DIRECTORY ${SWIG_OUT_ORIG}
                COMMENT "Creating pydoc..."
            )
        endif()
    endif()

    # ################# Install files ################
    configure_file(__init__.py.in
                   ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}/__init__.py)

    install(
        FILES ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}/${RWSIM_MODULE}.py
        DESTINATION "${PYTHON_INSTALL_DIR}/${RWSIM_MODULE}"
    )
    install(
        FILES ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RWSIM_MODULE}/__init__.py
        DESTINATION "${PYTHON_INSTALL_DIR}/${RWSIM_MODULE}"
    )
    # ###### Final linking and dependency checks ######

    target_link_libraries(${python_NAME_sdurwsim} PUBLIC sdurwsim RW::sdurw RWS::sdurws ${RWSIM_ODE_LIBRARY}
                        ${PYTHON_LIBRARIES})
                        
    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}Targets)

    RW_SWIG_COMPILE_TARGET(
            python ${TARGET_NAME}
        )

endif()
