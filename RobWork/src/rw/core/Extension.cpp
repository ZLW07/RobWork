#include "Extension.hpp"

#include <rw/core/Plugin.hpp>

using namespace rw::core;

Extension::Extension (const std::string& id, const std::string& point, Plugin* owner) :
    _desc (id, point), _owner (owner)
{}

Extension::Extension (const std::string& id, const std::string& point, Plugin* owner,
                      rw::core::AnyPtr obj) :
    _desc (id, point),
    _owner (owner), _obj (obj)
{}

Extension::Extension (Descriptor desc, Plugin* owner) : _desc (desc), _owner (owner)
{}
