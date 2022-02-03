#include "Lua.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    int luaopen_sdurwsim (lua_State* L);    // declare the wrapped module

#ifdef __cplusplus
}
#endif

int rwsim::swig::openLuaLibRWSim (lua_State* L)
{
    return luaopen_sdurwsim (L);
}
