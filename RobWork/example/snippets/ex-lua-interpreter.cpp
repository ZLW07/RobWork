#include <iostream>
#include <rwlibs/swig/lua/Lua_sdurw.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_assembly.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_control.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_pathoptimization.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_pathplanners.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_proximitystrategies.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_simulation.hpp>
#include <rwlibs/swig/lua/Lua_sdurw_task.hpp>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <lua-script>\n";
        return 1;
    }

    lua_State *L = luaL_newstate();
    luaL_openlibs(L);

    rwlibs::swig::openLuaLibRW_sdurw(L);
    rwlibs::swig::openLuaLibRW_sdurw_assembly(L);
    rwlibs::swig::openLuaLibRW_sdurw_control(L);
    rwlibs::swig::openLuaLibRW_sdurw_pathoptimization(L);
    rwlibs::swig::openLuaLibRW_sdurw_pathplanners(L);
    rwlibs::swig::openLuaLibRW_sdurw_proximitystrategies(L);
    rwlibs::swig::openLuaLibRW_sdurw_simulation(L);
    rwlibs::swig::openLuaLibRW_sdurw_task(L);

    const int error = luaL_dofile(L, argv[1]);
    if (error) std::cerr << lua_tostring(L, -1) << "\n";
    lua_close(L);

    return error;
}
