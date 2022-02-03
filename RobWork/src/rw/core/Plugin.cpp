#include "Plugin.hpp"

#include <RobWorkConfig.hpp>
#include <rw/core/DOMCorePropertyMapLoader.hpp>
#include <rw/core/DOMElem.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/core/StringUtil.hpp>

#include <boost/filesystem.hpp>
#include <iostream>

#ifdef RW_WIN32
#include <windows.h>
// Must winbase.h be below windows.h
#include <winbase.h>
#else
#include <dlfcn.h>
#endif

using namespace rw::core;

struct Plugin::OSHandle
{
#ifdef RW_WIN32
    OSHandle (HINSTANCE handle) : handle (handle) {}
    HINSTANCE handle;
#else
    OSHandle (void* handle) : handle (handle) {}
    void* handle;
#endif
};

namespace {
class AutoClosePlugin : public Plugin
{
  public:
    AutoClosePlugin (Plugin* plugin) :
        Plugin (plugin->getId (), plugin->getName (), plugin->getVersion ()), _plugin (plugin)
    {}

    ~AutoClosePlugin ()
    {
        const OSHandle* const handle = _plugin->getHandle ();
        delete _plugin;
        Plugin::close (handle);
    }

    std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ()
    {
        return _plugin->getExtensionDescriptors ();
    }

    rw::core::Ptr< Extension > makeExtension (const std::string& id)
    {
        return _plugin->makeExtension (id);
    }

  private:
    Plugin* _plugin;
};
}    // namespace

Plugin::Plugin (const std::string& id, const std::string& name, const std::string& version) :
    _id (id), _name (name), _version (version), _handle (NULL)
{}
Plugin::~Plugin ()
{}

std::vector< std::string > Plugin::getExtensionPointIDs ()
{
    std::vector< std::string > ids;
    const std::vector< Extension::Descriptor > desc = getExtensionDescriptors ();
    for (std::size_t i = 0; i < desc.size (); i++)
        ids.push_back (desc[i].id);
    return ids;
}

rw::core::Ptr< Plugin > Plugin::load (const std::string& filename)
{
    boost::filesystem::path file (filename);
    if (!exists (file)) {
        RW_THROW ("The file does not exist: " << filename);
    }

#if (BOOST_FILESYSTEM_VERSION == 2)
    if (rw::core::StringUtil::toUpper (file.extension ()) == ".XML") {
#else
    if (rw::core::StringUtil::toUpper (file.extension ().string ()) == ".XML") {
#endif
        // IF file is an xml file
        return loadLazy (filename);
    }
    return loadDirect (filename);
}

const Plugin::OSHandle* Plugin::getHandle ()
{
    return _handle;
}

#ifdef RW_WIN32

void Plugin::close (const OSHandle* handle)
{
    if (handle != NULL) {
        /*if (!FreeLibrary(handle->handle)) {
        LPTSTR buffer = NULL;
        if(FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM,
                      NULL,             // Instance
                      GetLastError(),   // Message Number
                      0,                // Language
                      buffer,              // Buffer
                      0,                // Min/Max Buffer size
                      NULL))            // Arguments
        {
                RW_THROW("Error: Plugin could not be unloaded: " << buffer << "!");
        } else {
                RW_THROW("Unknown Error: Plugin could not be unloaded!");
        }
        }*/
        delete handle;
    }
}

rw::core::Ptr< Plugin > Plugin::loadDirect (const std::string& filename)
{
    HINSTANCE h = LoadLibraryA (filename.c_str ());
    if (h == NULL) {
        LPTSTR buffer = NULL;
        std::cout << "Ready to generate error message " << GetLastError () << std::endl;
        if (FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
                           NULL,               // Instance
                           GetLastError (),    // Message Number
                           0,                  // Language
                           buffer,             // Buffer
                           0,                  // Min/Max Buffer size
                           NULL))              // Arguments
        {
            RW_THROW (buffer);
        }
        else if (GetLastError () == 126 ){
            RW_THROW ("Error 126: Error loading dependent dll");
        }
        else {
            RW_THROW ("Unknown Error: Could not open library");
        }
    }

    // try extract a symbol from the library
    // get any error message if there is any
    void* (*factory_func) (void) = NULL;
    void** tmp                   = (void**) &factory_func;
    const char* sym_name         = "createplugin";
    *tmp                         = (void*) GetProcAddress (h, sym_name);
    if (*tmp == NULL) {
        LPTSTR buffer     = NULL;
        const DWORD errId = GetLastError ();
        const DWORD res =
            FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                               FORMAT_MESSAGE_IGNORE_INSERTS,
                           NULL,               // Instance
                           errId,              // Message Number
                           0,                  // Language
                           (LPSTR) &buffer,    // Buffer
                           0,                  // Min/Max Buffer size
                           NULL);              // Arguments
        if (!res) {
            RW_WARN ("FormatMessage returned error: " << GetLastError () << "!");
        }
        else {
            RW_WARN ("Error " << errId << ": " << buffer);
        }
        return NULL;
    }

    // call factory function and create plugin
    Plugin* lplugin  = (Plugin*) factory_func ();
    lplugin->_handle = new OSHandle (h);
    return rw::core::ownedPtr (new AutoClosePlugin (lplugin));
}

#else    //#ifdef RW_WIN32

void Plugin::close (const OSHandle* handle)
{
    if (handle != NULL) {
        const int close = dlclose (handle->handle);
        if (close != 0) {
            char* err = dlerror ();
            if (err != NULL)
                RW_THROW ("Error: Could not close library: " << err);
            RW_THROW ("Error: Plugin could not be unloaded!");
        }
        delete handle;
    }
}

rw::core::Ptr< Plugin > Plugin::loadDirect (const std::string& filename)
{
    void* handle = dlopen (filename.c_str (), RTLD_NOW /* RTLD_GLOBAL*/);
    char* err = dlerror ();

    if (handle == NULL || err != NULL)
        RW_THROW ("Unknown Error: Could not open library: " << err);

    if (err != 0) {
        return NULL;
    }

    const char* func = "createplugin";

    // if (getSymbol((void**)&factory_func, func )) {

    void* (*factory_func) (void) = NULL;
    void** tmp = (void**) &factory_func;
    *tmp = dlsym (handle, func);
    err = dlerror ();
    if (err == 0) {
        Plugin* lplugin = (Plugin*) factory_func ();
        lplugin->_handle = new OSHandle (handle);
        return rw::core::ownedPtr (new AutoClosePlugin (lplugin));
    }
    else {
        RW_THROW (
            "Error: Plugin is not valid! Unable to identify factory function in dynamic library: "
            << err);
    }
    return NULL;
}

#endif    //#else

namespace {
class LazyPlugin : public Plugin
{
  public:
    LazyPlugin (const std::string& id, const std::string& name, const std::string& version) :
        Plugin (id, name, version)
    {}

    void setExtensionDescriptors (std::vector< Extension::Descriptor > descs) { _descs = descs; }

    void setLibFile (const std::string& file) { _libfile = file; }

    std::vector< Extension::Descriptor > getExtensionDescriptors () { return _descs; }

    rw::core::Ptr< Extension > makeExtension (const std::string& id)
    {
        Extension::Descriptor* desc = NULL;
        for (Extension::Descriptor& desc_tmp : _descs) {
            // std::cout << desc_tmp.id << "==" << id << std::endl;
            if (desc_tmp.id == id) {
                desc = &desc_tmp;
                break;
            }
        }
        if (desc == NULL)
            RW_THROW ("Not a valid id!");
        // now comes the loading part
        if (_srcPlugin == NULL)
            _srcPlugin = Plugin::load (_libfile);

        _descs = _srcPlugin->getExtensionDescriptors ();
        return _srcPlugin->makeExtension (id);
    }

  private:
    std::vector< Extension::Descriptor > _descs;
    rw::core::Ptr< Plugin > _srcPlugin;
    std::string _libfile;
};
}    // namespace

rw::core::Ptr< Plugin > Plugin::loadLazy (const std::string& filename)
{
    // parse xml file
    const DOMParser::Ptr parser = DOMParser::Factory::getDOMParser ("XML");
    parser->load (filename);
    const DOMElem::Ptr root   = parser->getRootElement ();
    const DOMElem::Ptr plugin = root->getChild ("plugin");
    const std::string id      = plugin->getAttributeValue ("id");
    const std::string name    = plugin->getAttributeValue ("name");
    const std::string version = plugin->getAttributeValue ("version");

    const std::string runtimelib = plugin->getChild ("runtime")->getAttributeValue ("library");
    boost::filesystem::path libfile (runtimelib);

#if (BOOST_FILESYSTEM_VERSION == 2)
    if (!libfile.has_root_path ()) {
        std::string fname =
            boost::filesystem::path (filename).parent_path ().string () + SLASH + runtimelib;
        libfile = boost::filesystem::path (fname);
    }
#else
    if (!libfile.is_absolute ()) {
        std::string fname =
            boost::filesystem::path (filename).parent_path ().string () + SLASH + runtimelib;
        libfile = boost::filesystem::path (fname);
    }
#endif

    if (!exists (libfile))
        RW_THROW ("The plugin file specified in \n" << filename << "\n does not exist.");

    std::vector< Extension::Descriptor > ext_descriptors;
    for (DOMElem::Iterator p = plugin->begin (); p != plugin->end (); ++p) {
        if (p->getName () == "extension") {
            Extension::Descriptor extension;
            extension.id    = p->getAttributeValue ("id");
            extension.name  = p->getAttributeValue ("name");
            extension.point = p->getAttributeValue ("point");
            DOMElem::Iterator child;
            for (child = p->begin (); child != p->end (); ++child) {
                if (child->getName () == "PropertyMap") {
                    extension.props = DOMCorePropertyMapLoader::readProperties (*child);
                    break;
                }
            }
            ext_descriptors.push_back (extension);
        }
    }

    LazyPlugin* lplugin = new LazyPlugin (id, name, version);
    lplugin->setLibFile (libfile.string ());
    lplugin->setExtensionDescriptors (ext_descriptors);
    return rw::core::ownedPtr (new AutoClosePlugin (lplugin));
}
