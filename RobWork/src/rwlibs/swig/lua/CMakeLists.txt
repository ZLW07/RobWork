set(SUBSYS_NAME sdurw_lua)
set(SUBSYS_DESC "Interface for accessing RobWork from lua.")
set(SUBSYS_DEPS)

set(build TRUE)

if(NOT RW_HAVE_LUA)
    set(DEFAULT false)
    set(REASON "Lua not found!")
else()
    set(DEFAULT true)
    set(REASON "Configured by default.")
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseSWIG)

    set(CMAKE_SWIG_FLAGS "-v -MM")

    add_library(${SUBSYS_NAME}_all INTERFACE)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME}_all ALIAS ${SUBSYS_NAME}_all)

    include_directories(${RW_ROOT}/src)
    set(lua_targets)
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
        sdurw_proximitystrategies
        sdurw_pathoptimization
        sdurw_simulation
        sdurw_opengl
        sdurw_pathplanners
    )
        if(TARGET ${RW_MODULE})
            configure_file(Lua.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/Lua_${RW_MODULE}.hpp @ONLY)
            configure_file(Lua.cpp.in Lua_${RW_MODULE}.cpp @ONLY)
            configure_file(LuaPlugin.cpp.in LuaPlugin_${RW_MODULE}.cpp @ONLY)

            set(SOURCE)
            if("${RW_MODULE}" STREQUAL "sdurw")
                set(SOURCE ../ScriptTypes.cpp)
            endif()
            if("${RW_MODULE}" STREQUAL "sdurw_core")
                set(SOURCE ${SOURCE} LuaState.cpp)
            endif()
            set(SOURCE ${SOURCE} Lua_${RW_MODULE}.cpp)

            # ############ lua interface generation ##############

            set(TARGET_NAME ${RW_MODULE}_lua)
            rw_add_swig(
                ${RW_MODULE} lua MODULE
                TARGET_NAME ${TARGET_NAME}
                SOURCES ${SOURCE}
                INSTALL_DIR ${LUA_INSTALL_DIR} BINARY_OUTPUT_DIR
                                               ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua
            )
            
            set(TARGET_NAME ${lua_NAME_${RW_MODULE}})
            if(MSVC)
                set_target_properties(
                    ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF
                                              WINDOWS_EXPORT_ALL_SYMBOLS TRUE
                )
            else()
                set_target_properties(${TARGET_NAME} PROPERTIES PREFIX "" OUTPUT_NAME ${RW_MODULE})
            endif()

            set(lua_targets ${lua_targets} ${TARGET_NAME})

            if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
                target_link_libraries(${SUBSYS_NAME}_all INTERFACE ${TARGET_NAME})
            endif()

            # ############ lua static interface generation ##############
            rw_add_swig(
                ${RW_MODULE} lua STATIC
                TARGET_NAME ${TARGET_NAME}_s
                SOURCES ${SOURCE}
                INSTALL_DIR ${STATIC_LIB_INSTALL_DIR}
            )

            if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
                target_link_libraries(${SUBSYS_NAME}_all INTERFACE ${TARGET_NAME}_s)
            endif()

            set(lua_targets ${lua_targets} ${TARGET_NAME}_s)

            # this is used to indicate static linking to Visual Studio or mingw
            if(DEFINED MSVC)
                set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
            else()
                set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
            endif()

            # ############### Add rwplugin #####################
            if(NOT "${RW_MODULE}" STREQUAL "sdurw"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_core"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_common"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_math"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_plugin"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_geometry"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_sensor"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_models"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_proximity"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_graphics"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_trajectory"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_invlin"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_pathplanning"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_graspplanning"
               AND NOT "${RW_MODULE}" STREQUAL "sdurw_loaders"
            )
                add_library(${RW_MODULE}_lua_plugin.rwplugin MODULE LuaPlugin_${RW_MODULE}.cpp)
                target_link_libraries(${RW_MODULE}_lua_plugin.rwplugin ${lua_s_NAME_${RW_MODULE}})
                add_dependencies(${RW_MODULE}_lua_plugin.rwplugin ${lua_s_NAME_${RW_MODULE}})

                if(NOT SWIG_DEFAULT_COMPILE)
                    set_target_properties(
                        ${RW_MODULE}_lua_plugin.rwplugin PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE
                    )
                endif()
                set(lua_targets ${lua_targets} ${RW_MODULE}_lua_plugin.rwplugin)

                target_include_directories(
                    ${RW_MODULE}_lua_plugin.rwplugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${LUA_INCLUDE_DIR}
                )
                if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
                    set_target_properties(
                        ${RW_MODULE}_lua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined
                    )
                endif()

                if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
                    install(TARGETS ${RW_MODULE}_lua_plugin.rwplugin
                            LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR}
                    )
                endif()
            endif()


            # ############### Linking #####################
            get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})
            target_link_libraries(${RW_MODULE}_lua  PUBLIC ${RW_MODULE} ${depends} ${LUA_LIBRARIES})
            if(CMAKE_VERSION VERSION_GREATER 3.12.0)
                list(TRANSFORM depends APPEND "_lua_s")
            else()
                set(cpd)
                foreach(dep ${depends})
                    set(cpd ${cpd} "${dep}_lua_s")
                    set(cpd2 ${cpd} "${dep}_lua")
                endforeach()
                set(depends "${cpd}")
                set(depends2 "${cpd}")
            endif()
            if(${RW_MODULE} STREQUAL "sdurw_core")
                target_link_libraries(${RW_MODULE}_lua_s PUBLIC ${RW_MODULE} ${depends} ${LUA_LIBRARIES})
            else()
                target_link_libraries(
                    ${RW_MODULE}_lua_s PUBLIC ${RW_MODULE} ${depends} sdurw_core_lua_s ${LUA_LIBRARIES}
                )
            endif()

            get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})
            foreach(dep ${depends})
                set(SWIG_MODULE_${TARGET_NAME}_EXTRA_DEPS ${SWIG_MODULE_${TARGET_NAME}_EXTRA_DEPS} ${lua_NAME_${dep}})
                set(SWIG_MODULE_${TARGET_NAME}_s_EXTRA_DEPS ${SWIG_MODULE_${TARGET_NAME}_s_EXTRA_DEPS} ${lua_s_NAME_${dep}})
                add_dependencies(${TARGET_NAME} ${lua_NAME_${dep}})                
                add_dependencies(${TARGET_NAME}_s ${lua_s_NAME_${dep}}) 
                if(TARGET ${RW_MODULE}_lua_plugin.rwplugin AND TARGET ${dep}_lua_plugin.rwplugin)
                    add_dependencies(${RW_MODULE}_lua_plugin.rwplugin ${dep}_lua_plugin.rwplugin)
                endif()
            endforeach()

            add_dependencies(${TARGET_NAME}_s ${TARGET_NAME})
            
            target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${LUA_INCLUDE_DIR})
            target_include_directories(${TARGET_NAME}_s PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${LUA_INCLUDE_DIR})

        endif()

        get_in_map(depends RW_SUBSYS_DEPEND ${RW_MODULE})
        foreach(dep ${depends})
            add_dependencies(${TARGET_NAME} ${lua_NAME_${dep}})
        endforeach()

    endforeach()

    install(TARGETS ${SUBSYS_NAME}_all EXPORT ${PROJECT_PREFIX}luaTargets)

    rw_swig_compile_target(lua ${lua_targets})

endif()
