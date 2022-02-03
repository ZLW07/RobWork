# ##################################################################################################
# Rcc shared for all plugins
#
set(RccSrcFiles resources.qrc)

# ##################################################################################################
# next compile RWSimulatorPlugin
#
set(SUBSYS_NAME RWSimulatorPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS RWS::sdurws RW::sdurw_simulation RW::sdurw_opengl RW::sdurw sdurwsim)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles RWSimulatorPlugin.cpp)
    set(HppFiles RWSimulatorPlugin.hpp)

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${HppFiles} ${RccSrcFiles})
    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim
        PUBLIC RWS::sdurws
        PRIVATE RW::sdurw_simulation RW::sdurw_opengl
        PUBLIC RW::sdurw
    )

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# next compile SDHPlugin
#
# include_directories(${ROOT}/../../RobWorkHardware/src/)
# link_directories(${ROOT}/../../RobWorkHardware/libs/Release/)
# link_directories(${ROOT}/../../RobWorkHardware/ext/sdh/libs/) link_directories("C:/Program
# Files/ESD/CAN/SDK/lib/vc/i386")

# SET(SrcFiles SDHPlugin.cpp ) SET(MocSrcFiles SDHPlugin.cpp ) SET(MocHeaderFiles SDHPlugin.hpp )
# SET(QrcFiles resources.qrc)

# Call the create_plugin macro for creating the plugin ADD_PLUGIN(SDHPlugin SrcFiles MocHeaderFiles
# QrcFiles) TARGET_LINK_LIBRARIES(SDHPlugin ${ROBWORKSIM_LIBRARIES} sdurwhw_sdh sdh ntcan.lib)

# ##################################################################################################
# next compile SimUtilityPlugin
#
set(SUBSYS_NAME SimUtilityPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles SimUtilityPlugin.cpp)

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles SimUtilityPlugin.hpp )

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui sdurwsim
        PUBLIC RWS::sdurws RW::sdurw
    )
    if(MSVC) # to avoid multiple access to resource file
        add_dependencies(${SUBSYS_NAME} RWSimulatorPlugin)
    endif()

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

# ##################################################################################################
# next compile GraspTableGeneratorPlugin
#
set(SUBSYS_NAME GraspTableGeneratorPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(SrcFiles GraspTableGeneratorPlugin.cpp)
    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles GraspTableGeneratorPlugin.hpp)

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw)

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

    if(MSVC) # to avoid multiple access to resource file
        add_dependencies(${SUBSYS_NAME} SimUtilityPlugin)
    endif()
endif()

# ##################################################################################################
# The RW Simulation
#
set(SUBSYS_NAME RWSimPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS
    sdurwsim_gui
    sdurwsim
    RWS::sdurws
    RW::sdurw_opengl
    RW::sdurw_simulation
    RW::sdurw_control
    RW::sdurw
)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles RWSimPlugin.cpp)

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles RWSimPlugin.hpp )
    set(RccSrcFiles_RWSimPlugin ${RWSIM_ROOT}/src/gfx/RWSimPlugin.qrc)

    set(RWSIM_LUA_LIB_TMP RW::sdurw_lua_s sdurwsim_lua_s)
    rw_is_targets(test TARGETS ${RWSIM_LUA_LIB_TMP} TARGETS_OUT RWSIM_LUA_LIB)

    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_RWSimPlugin})
    set_target_properties(${SUBSYS_NAME} PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)

    if(${test})
        target_compile_definitions(${SUBSYS_NAME} PRIVATE RWSIM_PLUGIN_HAVE_LUA)
    endif()

    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui
        PUBLIC sdurwsim
        PRIVATE ${RWSIM_LUA_LIB}
        PUBLIC RWS::sdurws
        PRIVATE RW::sdurw_opengl RW::sdurw_simulation RW::sdurw_control
        PUBLIC RW::sdurw
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(RWSimPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(RWSimPlugin PRIVATE ${PYTHON_INCLUDE_DIRS})

    if(MSVC) # to avoid multiple access to resource file
        add_dependencies(${SUBSYS_NAME} GraspTableGeneratorPlugin)
    endif()
endif()

# ##################################################################################################
# The SimTaskPlugin
#
set(SUBSYS_NAME SimTaskPlugin)
set(SUBSYS_DESC "A RobWorkStudioPlugin.")
set(SUBSYS_DEPS sdurwsim_gui sdurwsim RWS::sdurws RW::sdurw_task RW::sdurw_opengl RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(SrcFiles SimTaskPlugin.cpp)
    set(MocSrcFiles SimTaskPlugin.hpp)
    set(RccSrcFiles_SimTaskPlugin ${RccSrcFiles} ${RWSIM_ROOT}/src/gfx/SimTaskPlugin.qrc)

    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles_SimTaskPlugin})

    target_link_libraries(
        ${SUBSYS_NAME}
        PRIVATE sdurwsim_gui
        PUBLIC sdurwsim RWS::sdurws
        PRIVATE RW::sdurw_task RW::sdurw_opengl
        PUBLIC RW::sdurw
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

    if(MSVC) # to avoid multiple access to resource file
        add_dependencies(${SUBSYS_NAME} RWSimPlugin)
    endif()
endif()

# ##################################################################################################
# The SimTaskVisPlugin -- has been moved to RobWorkStudio now named gtaskplugin
#

# ##################################################################################################
# next compile EngineTestPlugin
#

set(SUBSYS_NAME EngineTestPlugin)
set(SUBSYS_DESC "A plugin for executing standard tests for Physics Engines.")
set(SUBSYS_DEPS sdurwsim_gui RWS::sdurws)

set(build TRUE)
set(DEFAULT TRUE)

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(SrcFiles EngineTestPlugin.cpp)

    # Call the create_plugin macro for creating the plugin
    set(MocSrcFiles EngineTestPlugin.hpp)

    # The shared library to build:
    add_library(${SUBSYS_NAME} MODULE ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC sdurwsim_test
        PRIVATE sdurwsim_gui
        PUBLIC RWS::sdurws
    )

    # Need to add the current binary dir to the include directory because the generated source files
    # are placed here
    target_include_directories(${SUBSYS_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # to avoid windows compile bug
    target_include_directories(${SUBSYS_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

    if(MSVC) # to avoid multiple access to resource file
        add_dependencies(${SUBSYS_NAME} SimTaskPlugin)
    endif()
endif()
