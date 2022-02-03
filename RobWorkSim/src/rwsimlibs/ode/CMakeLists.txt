set(SUBSYS_NAME sdurwsim_ode)
set(SUBSYS_DESC "Open Dynamics Engine library for RobWorkSim")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)
set(DEFAULT ${RWSIM_HAVE_ODE})
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(Ext ${PROJECT_SOURCE_DIR}/ext)

    set(
        SRC_FILES
        ODESimulator.cpp
        ODEVelocityDevice.cpp
        ODEKinematicDevice.cpp
        ODESuctionCupDevice.cpp
        ODEBody.cpp
        ODEJoint.cpp
        ODEUtil.cpp
        ODEDebugRender.cpp
        ODETactileSensor.cpp
        ODEMaterialMap.cpp
        ODESuctionCupDevice.cpp
        ODEConstraint.cpp
        ODEContactStrategy.cpp
        ODEThreading.cpp
        ODELogUtil.cpp
    )

    set(
        SRC_FILES_HPP
        ODESimulator.hpp
        ODEVelocityDevice.hpp
        ODEKinematicDevice.hpp
        ODESuctionCupDevice.hpp
        ODEBody.hpp
        ODEJoint.hpp
        ODEUtil.hpp
        ODEDebugRender.hpp
        ODEDevice.hpp
        ODETactileSensor.hpp
        ODEMaterialMap.hpp
        ODEConstraint.hpp
        ODEContactStrategy.hpp
        ODEThreading.hpp
        ODELogUtil.hpp
    )

    # Detect if ODE comes with threading_impl.h file (version 0.13 and newer).

    if(NOT DEFINED ODE_WITH_THREADING_IMPL)
        message(STATUS "Testing ODE for the threading_impl.h file...")
        if(EXISTS "${ODE_INCLUDE_DIR}/ode/threading_impl.h")
            message(STATUS "ODE appears to have threading_impl.h file (version 0.13 or newer).")
            set(
                ODE_WITH_THREADING_IMPL
                TRUE
                CACHE BOOL "Select if ODE has threading_impl.h file (version 0.13 and newer)."
            )
        else()
            message(STATUS "ODE does not appear to have threading_impl.h file (version 0.12 or earlier).")
            set(
                ODE_WITH_THREADING_IMPL
                FALSE
                CACHE BOOL "Select if ODE does not have threading_impl.h file (version 0.12 or earlier)."
            )
        endif()
    endif()

    if(${ODE_WITH_THREADING_IMPL})
        message(STATUS "ODE threading_impl.h detected - assuming version 0.13 and later.")
        set_source_files_properties(ODEThreading.cpp PROPERTIES COMPILE_DEFINITIONS ODE_WITH_THREADING_IMPL)
    elseif()
        message(STATUS "ODE threading_impl.h NOT detected - assuming version 0.12 and earlier.")
    endif()

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/ode" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwsim ${ODE_LIBRARIES} RW::sdurw ${OPENGL_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWSIM_ROOT}/src>
        $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        PUBLIC
        $<BUILD_INTERFACE:${ODE_INCLUDE_DIR}> 
    )

    set(SUBSYS_NAME_PLUGIN ode_plugin.rwplugin)
    set(SUBSYS_DESC_PLUGIN "Plugin providing Open Dynamics Engine.")
    set(SUBSYS_DEPS_PLUGIN ${SUBSYS_NAME})

    set(build_plugin TRUE)
    rw_subsys_option(build_plugin ${SUBSYS_NAME_PLUGIN} ${SUBSYS_DESC_PLUGIN} ${DEFAULT} DEPENDS ${SUBSYS_DEPS_PLUGIN} ADD_DOC)

    if(build_plugin)
        rw_add_plugin(${SUBSYS_NAME_PLUGIN} MODULE ODEPlugin.cpp ODEPlugin.hpp)
        target_link_libraries(${SUBSYS_NAME_PLUGIN} PRIVATE sdurwsim_ode)
        install(TARGETS ${SUBSYS_NAME_PLUGIN} LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR})
    endif()

    if(NOT ODE_WITH_THREADING_IMPL) # version below 0.13
        if("${ODE_BUILD_WITH}" STREQUAL "DOUBLE")
            target_compile_definitions(sdurwsim_ode PUBLIC dDOUBLE)
        else()
            target_compile_definitions(sdurwsim_ode PUBLIC dSINGLE)
        endif()
    endif()

endif()
