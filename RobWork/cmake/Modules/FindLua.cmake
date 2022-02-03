list(REMOVE_ITEM CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules)
find_package(Lua QUIET)
set(CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

if(NOT LUA_FOUND)
    find_path(
        LUA_INCLUDE_DIR
        NAMES lua.hpp
        PATHS "$ENV{Lua_ROOT}/"
              "$ENV{LUA_ROOT}/"
              "/usr"
              "C:/Local/Lua"
              "C:/Program Files/Lua"
              "C:/Program Files (x86)/Lua"
              "C:/Local/lua"
              "C:/Program Files/lua"
              "C:/Program Files (x86)/lua"
        PATH_SUFFIXES "include" "src"
    )
    find_library(
        LUA_LIBRARIES
        lua
        lua5
        lua51
        lua52
        lua53
        lua54
        PATHS "$ENV{Lua_ROOT}/"
              "$ENV{LUA_ROOT}/"
              "/usr"
              "C:/Local/Lua"
              "C:/Program Files/Lua"
              "C:/Program Files (x86)/Lua"
              "C:/Local/lua"
              "C:/Program Files/lua"
              "C:/Program Files (x86)/lua"
        PATH_SUFFIXES "bin" "lib"
    )

    include(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set Lua_FOUND to TRUE if all listed variables
    # are TRUE
    find_package_handle_standard_args(Lua DEFAULT_MSG LUA_LIBRARIES LUA_INCLUDE_DIR)

    mark_as_advanced(LUA_INCLUDE_DIR LUA_LIBRARIES)
endif()
