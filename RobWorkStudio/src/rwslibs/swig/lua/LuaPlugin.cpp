#include "LuaPlugin.hpp"

#include "Lua.hpp"

#include <rwlibs/swig/lua/LuaState.hpp>

using namespace rwslibs::swig;
using namespace rw::core;

RW_ADD_PLUGIN (LuaPlugin)

LuaPlugin::LuaPlugin () : Plugin ("RWSLuaPlugin", "RWSLuaPlugin", "0.1")
{}

LuaPlugin::~LuaPlugin ()
{}

std::vector< rw::core::Extension::Descriptor > LuaPlugin::getExtensionDescriptors ()
{
    std::vector< Extension::Descriptor > exts;
    exts.push_back (Extension::Descriptor ("RWSLua", "rwlibs.swig.LuaState.LuaLibrary"));

    // todo: add posible properties to the extension descriptor
    exts.back ().getProperties ().set< std::string > ("ID", "rws");
    // exts.back().getProperties().set<std::string>("engineID", "ODE");

    return exts;
}
namespace {
struct RWSLuaLibrary : rwlibs::swig::LuaState::LuaLibrary
{
    virtual const std::string getId () { return "RWSLua"; }
    virtual bool initLibrary (rwlibs::swig::LuaState& state)
    {
        std::cout << "INIT rws LUALIB" << std::endl << std::flush;
        rwslibs::swig::openLuaLibRWS (state.get ());
        // initialize variables

        state.runCmd ("rws = rws.lua.rwstudio");

        state.runCmd ("rwstudio = rws.getRobWorkStudio()");

        return true;
    };
};
}    // namespace
rw::core::Ptr< rw::core::Extension > LuaPlugin::makeExtension (const std::string& str)
{
    if (str == "RWSLua") {
        Extension::Ptr extension = rw::core::ownedPtr (new Extension (
            "RWSimLua", "rwlibs.swig.LuaState.LuaLibrary", this, ownedPtr (new RWSLuaLibrary ())));

        // todo: add posible properties to the extension descriptor
        // exts.back().getProperties().set<std::string>(propid, value);
        extension->getProperties ().set< std::string > ("ID", "rws");
        return extension;
    }
    return NULL;
}
