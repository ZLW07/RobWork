#include "Lua.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    int luaopen_sdurws (lua_State* L);    // declare the wrapped module

#ifdef __cplusplus
}
#endif

int rwslibs::swig::openLuaLibRWS (lua_State* L)
{
    return luaopen_sdurws (L);
}
