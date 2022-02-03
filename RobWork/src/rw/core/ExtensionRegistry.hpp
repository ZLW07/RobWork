/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_CORE_EXTENSIONREGISTRY_HPP
#define RW_CORE_EXTENSIONREGISTRY_HPP

#if !defined(SWIG)
#include <rw/core/Extension.hpp>
#include <rw/core/Ptr.hpp>

#include <map>
#endif
namespace rw { namespace core {

    /**
     * @brief an extension point is a class that defines a point where Extension can be added.
     * This is typically used together with plugins, however any class may register extensions
     * to an extension point.
     */
    class ExtensionRegistry
    {
      public:
        //! smart pointer type of ExtensionPoint
        typedef rw::core::Ptr< ExtensionRegistry > Ptr;

        //! @brief Constructor
        ExtensionRegistry ();

        //! @brief Destructor
        ~ExtensionRegistry ();

        //! get registry instance
        static rw::core::Ptr< ExtensionRegistry > getInstance ();

        /**
         * @brief get all descriptors registered for a specific extension point id
         * @param ext_point_id [in] identifier of extension point
         * @return list of extension point descriptions
         */
        std::vector< rw::core::Extension::Descriptor >
        getExtensionDescriptors (const std::string& ext_point_id) const;

        /**
         * @brief get all extensions of a specific extension point
         * @param ext_point_id [in] identifier of extension point
         * @return list of extensions
         */
        std::vector< rw::core::Ptr< Extension > >
        getExtensions (const std::string& ext_point_id) const;

        /**
         * @brief register extensions and extension points of a plugin
         * @param plugin [in] the plugin that is to be registered
         */
        void registerExtensions (rw::core::Ptr< Plugin > plugin);

        /**
         * @brief Unregister extensions and extension points of a plugin.
         * @param plugin [in] the plugin that is to be removed.
         */
        void unregisterExtensions (rw::core::Ptr< Plugin > plugin);

        //! @brief Unregister all extensions.
        void clearExtensions ();

        /**
         * @brief get a list of registered plugins
         * @return list of plugins
         */
        std::vector< rw::core::Ptr< Plugin > > getPlugins () const;

      private:
        // maps extension point id's into description-plugin pair
        std::map< std::string,
                  std::vector< std::pair< Extension::Descriptor, rw::core::Ptr< Plugin > > > >
            _descMap;

        std::list< rw::core::Ptr< Plugin > > _plugins;
    };

}}    // namespace rw::core

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    using namespace rw::core;
}}    // namespace rw::common

#endif
