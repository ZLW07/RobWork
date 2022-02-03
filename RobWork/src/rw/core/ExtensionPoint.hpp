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
#ifndef RW_CORE_EXTENSIONPOINT_HPP
#define RW_CORE_EXTENSIONPOINT_HPP

#if !defined(SWIG)
#include <rw/core/ExtensionRegistry.hpp>
#endif
namespace rw { namespace core {

    class Plugin;

    /**
     * @brief an extension point is a class that defines a point where Extension can be added.
     * This is typically used together with plugins, however any class may register extensions
     * to an extension point.
     */
    template< class ExtensionInterface > class ExtensionPoint
    {
      public:
        //! smart pointer type of ExtensionPoint
        typedef rw::core::Ptr< ExtensionPoint > Ptr;

        /**
         * @brief Constructor
         * @param id [in] unique identifier of this extension point
         * @param name [in] human readable name of this extension point
         * @param plugin [in] the plugin from which this extension point is defined, NULL if not
         * defined from plugin
         */
        ExtensionPoint (const std::string& id, const std::string& name, Plugin* plugin = NULL) :
            _id (id), _name (id), _owner (plugin)
        {}

        //! @brief get unique identifier of this extensionpoint
        const std::string& getId () const { return _id; }

        //! @brief get human readable name of this extension point
        const std::string& getName () const { return _name; }

        /**
         * @brief the schema describe the possible properties/configurations elements
         * which is used in the PropertyMap. It contain examples of all possible configuration
         * options. This can be used to configure any extensions that needs to attach to
         * this extension point.
         */
        const rw::core::PropertyMap& getSchema () const { return _schema; }

        //! @brief get all extension descriptions of this extension point
        std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors () const
        {
            ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance ();
            return reg->getExtensionDescriptors (_id);
        }

        //! @brief get all extensions of this extension point
        std::vector< rw::core::Ptr< Extension > > getExtensions () const
        {
            ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance ();
            return reg->getExtensions (_id);
        }

      protected:
        /**
         * @brief the schema describe the possible properties/configurations elements
         * which is used in the PropertyMap. The schema property map should just be loaded
         * with all possible configuration options which the extension might use.
         *
         * subclassing the ExtensionPoint class enables you to add extra configuration options.
         * This might be done as simply as this:
         *
         * \code
         * getSchema().add("SupportedFormats", "Comma seperated String of supported formats",
         * std::string("GIF,PNG,JPEG")); getSchema().add("ShowLights", "Should lights be on as
         * default?", true);
         * ...
         * \endcode
         *
         * which will enable the loading of these options from the plugin xml file descriptor
         *
         * \code
         * <plugin ... >
         * ...
         * <extension>
         * ...
         * <SupportedFormats>GIF,PNG,JPEG</SupportedFormats>
         * <ShowLights>false</ShowLights>
         * </extension>
         * </plugin>
         * \endcode
         *
         * @return
         */
        rw::core::PropertyMap& getSchema () { return _schema; };

      private:
        std::string _id, _name;
        Plugin* _owner;
        rw::core::PropertyMap _schema;
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
