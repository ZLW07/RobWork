set(SUBSYS_NAME sdurwsim_lua)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from lua.")
set(SUBSYS_DEPS RW::sdurw_lua_s RWS::sdurws_lua_s sdurwsim)

set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
    include(UseSWIG)

    include_directories(${RWSIM_ROOT}/src ${RW_ROOT}/src)

    set(RWSIM_MODULE sdurwsim)
    set(TARGET_NAME ${RWSIM_MODULE}_lua)

    # ############ lua interface generation ##############
    rw_add_swig(
        ${RWSIM_MODULE} lua MODULE 
        TARGET_NAME ${TARGET_NAME}
        SOURCES ../sdurwsim.i Lua.cpp ../ScriptTypes.cpp
        BINARY_OUTPUT_DIR "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Lua"
        INSTALL_DIR ${LUA_INSTALL_DIR}
    )


    if(NOT DEFINED MSVC) # This code is to make it easier to make debian packages
        set_target_properties(
            ${TARGET_NAME}
            PROPERTIES
                PREFIX ""
                OUTPUT_NAME ${RWSIM_MODULE}
        )
    endif()

    target_link_libraries(${TARGET_NAME} ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${RW_BUILD_WITH_LIBRARIES_LUA})
    target_include_directories(${TARGET_NAME} PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})



    # ############ lua static interface generation ##############
    rw_add_swig(
        ${RWSIM_MODULE} lua STATIC 
        TARGET_NAME ${TARGET_NAME}_s
        SOURCES ../sdurwsim.i Lua.cpp ../ScriptTypes.cpp
        INSTALL_DIR ${STATIC_LIB_INSTALL_DIR}
    )

    target_link_libraries(${TARGET_NAME}_s ${RWSIM_ODE_LIBRARY} ${RWSIM_MODULE} ${RW_BUILD_WITH_LIBRARIES_LUA})
    target_include_directories(${TARGET_NAME}_s PUBLIC ${RW_BUILD_WITH_LUA_INCLUDE_DIR})

    # this is used to indicate static linking to Visual Studio or mingw
    if(DEFINED MSVC)
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    else()
        set_target_properties(${TARGET_NAME}_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    endif()

    add_library(sdurwsimlua_plugin.rwplugin MODULE LuaPlugin.cpp LuaPlugin.hpp)

    if(NOT SWIG_DEFAULT_COMPILE)
        set_target_properties(sdurwsimlua_plugin.rwplugin PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    endif()

    target_link_libraries(sdurwsimlua_plugin.rwplugin ${TARGET_NAME}_s)

    if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
        install(TARGETS ${RWSIM_MODULE}lua_plugin.rwplugin DESTINATION ${RW_PLUGIN_INSTALL_DIR})
    endif()

    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurwsimlua_plugin.rwplugin PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()


    RW_SWIG_COMPILE_TARGET(lua ${TARGET_NAME} ${TARGET_NAME}_s sdurwsimlua_plugin.rwplugin)

endif()
