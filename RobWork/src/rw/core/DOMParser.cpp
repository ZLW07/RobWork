#include "DOMParser.hpp"

#include <rw/core/BoostXMLParser.hpp>
#include <rw/core/StringUtil.hpp>
#include <string>
#include <vector>

using namespace rw::core;

rw::core::Ptr< DOMParser > DOMParser::make ()
{
    return rw::core::ownedPtr (new BoostXMLParser ());
}

rw::core::Ptr< DOMParser > DOMParser::Factory::getDOMParser (const std::string& format)
{
    using namespace rw::core;
    DOMParser::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr ext : exts) {
        if (!ext->getProperties ().has (format))
            continue;
        // else try casting to DOMParser
        DOMParser::Ptr loader = ext->getObject ().cast< DOMParser > ();
        return loader;
    }
    if (StringUtil::toUpper (format) == "XML")
        return rw::core::ownedPtr (new BoostXMLParser ());
    RW_THROW ("No loader using that format exists...");
    return NULL;
}

bool DOMParser::Factory::hasDOMParser (const std::string& format)
{
    using namespace rw::core;

    if (StringUtil::toUpper (format) == "XML")
        return true;

    DOMParser::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        if (!ext.getProperties ().has (format))
            continue;
        return true;
    }
    return false;
}

std::vector< std::string > DOMParser::Factory::getSupportedFormats() {
    return std::vector<std::string>();
}