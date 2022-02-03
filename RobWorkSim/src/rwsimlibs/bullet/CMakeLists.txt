set(SUBSYS_NAME sdurwsim_bullet)
set(SUBSYS_DESC "Bullet based physics engine plugin for RobWorkSim")
set(SUBSYS_DEPS sdurwsim)

set(build FALSE)
set(DEFAULT TRUE)

if(NOT BULLET_FOUND)
    set(DEFAULT FALSE)
    set(REASON "Bullet not found!")
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(
        SRC_FILES
        BtUtil.cpp
        BtBody.cpp
        BtDevice.cpp
        BtPositionDevice.cpp
        BtVelocityDevice.cpp
        BtSimulator.cpp
        BtDebugRender.cpp
        BtConstraint.cpp
        BtTactileSensor.cpp
        BtMaterial.cpp
        BtContactStrategy.cpp
        BtRWCollisionAlgorithm.cpp
        BtRWCollisionConfiguration.cpp
    )

    set(
        SRC_FILES_HPP
        BtUtil.hpp
        BtBody.hpp
        BtDevice.hpp
        BtPositionDevice.hpp
        BtVelocityDevice.hpp
        BtSimulator.hpp
        BtDebugRender.hpp
        BtConstraint.hpp
        BtTactileSensor.hpp
        BtMaterial.hpp
        BtContactStrategy.hpp
        BtRWCollisionAlgorithm.hpp
        BtRWCollisionConfiguration.hpp
    )

    # Detect if Bullet is compiled in single or double precision by doing a test compilation
    set(
        BULLET_USE_SINGLE_PRECISION
        FALSE
        CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION."
    )
    set(
        BULLET_USE_DOUBLE_PRECISION
        FALSE
        CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION."
    )
    if(BULLET_USE_SINGLE_PRECISION)
        message(STATUS "Bullet will be compiled with single precision.")
    elseif(BULLET_USE_DOUBLE_PRECISION)
        message(STATUS "Bullet will be compiled with double precision.")
    else()
        message(STATUS "Testing Bullet for use of double precision...")
        set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
        try_compile(
            COMPILE_SUCCESS ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/BtPrecisionTest.cpp
            CMAKE_FLAGS
                "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
                "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
            COMPILE_DEFINITIONS "-DBT_USE_DOUBLE_PRECISION"
            OUTPUT_VARIABLE OUT
        )
        if(COMPILE_SUCCESS)
            message(STATUS "Bullet appears to be using double precision.")
            set(
                BULLET_USE_SINGLE_PRECISION
                OFF
                CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION." FORCE
            )
            set(
                BULLET_USE_DOUBLE_PRECISION
                ON
                CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." FORCE
            )
        else()
            message(STATUS "Bullet appears to be using single precision.")
            set(
                BULLET_USE_SINGLE_PRECISION
                ON
                CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION." FORCE
            )
            set(
                BULLET_USE_DOUBLE_PRECISION
                OFF
                CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." FORCE
            )
        endif()
    endif()

    if(BULLET_USE_DOUBLE_PRECISION)
        add_definitions(-DBT_USE_DOUBLE_PRECISION)
    endif()

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/bullet" ${SRC_FILES_HPP})

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} PRIVATE ${BULLET_LIBRARIES})

    target_include_directories(${SUBSYS_NAME} PRIVATE ${BULLET_INCLUDE_DIRS}
        INTERFACE
        $<BUILD_INTERFACE:${RWSIM_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_add_plugin(bt_plugin.rwplugin MODULE BtPlugin.cpp BtPlugin.hpp)
    target_link_libraries(bt_plugin.rwplugin PRIVATE sdurwsim_bullet)
    install(TARGETS bt_plugin.rwplugin LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR})

endif()
