/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWSIM_SWIG_LUAPLUGIN_HPP_
#define RWSIM_SWIG_LUAPLUGIN_HPP_

#include <rw/core/Extension.hpp>
#include <rw/core/Plugin.hpp>
#include <rw/core/Ptr.hpp>

namespace rwslibs { namespace swig {

    /**
     * @brief A Lua plugin that define extensions for rwlibs.swig.LuaState.LuaLibrary
     */
    class LuaPlugin : public rw::core::Plugin
    {
      public:
        /**
         * @brief constructor
         */
        LuaPlugin ();

        //! destructor
        virtual ~LuaPlugin ();

        //! @copydoc rw::core::Plugin::getExtensionDescriptors
        std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ();

        //! @copydoc rw::core::Plugin::makeExtension
        rw::core::Ptr< rw::core::Extension > makeExtension (const std::string& str);
    };

}}     // namespace rwslibs::swig
#endif /* LUA_HPP_ */
