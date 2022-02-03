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

#ifndef RW_CORE_EXTENSION_HPP
#define RW_CORE_EXTENSION_HPP

#if !defined(SWIG)
#include <rw/core/AnyPtr.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>

#include <string>
#endif
namespace rw { namespace core {
    class Plugin;

    /**
     * @brief The Extension class is used to provide additonal functionality from a
     * Plugin to other extension points of either the system or other plugins.
     *
     * %Extension points in %RobWork:
     * @copydetails extensionpoints
     */
    class Extension
    {
      public:
        //! @brief Smart pointer type for Extension.
        typedef rw::core::Ptr< Extension > Ptr;

        /**
         * @brief An extension descriptor.
         *
         * The descriptor holds meta-data for an extension, and makes it possible to do lazy-loading
         * of plugins.
         */
        struct Descriptor
        {
            //! @brief Construct empty descriptor.
            Descriptor () {}

            /**
             * @brief Construct new descriptor.
             * @param id_ [in] a unique id of the extension.
             * @param point_ [in] the extension point.
             */
            Descriptor (const std::string& id_, const std::string& point_) :
                id (id_), point (point_)
            {}

            //! @brief A unique id of an extension.
            std::string id;
            //! @brief A human-readable name of an extension.
            std::string name;
            //! @brief The extension point that this extension extends.
            std::string point;
            //! @brief Properties of the extension.
            rw::core::PropertyMap props;

            /**
             * @brief Get the extension properties.
             *
             * The properties will typically include information about the file extensions supported
             * by the plugin, or other meta-data required to determine the proper extension to use.
             *
             * @return a reference to the properties.
             */
            rw::core::PropertyMap& getProperties () { return props; }
#if !defined(SWIG)
            //! @copydoc getProperties
            const rw::core::PropertyMap& getProperties () const { return props; }
#endif
        };

      public:
        /**
         * @brief Constructor.
         * @param desc [in] Description and configuration of extension.
         * @param owner [in] the plugin that owns this extension or NULL.
         */
        Extension (Descriptor desc, Plugin* owner);

        /**
         * @brief Constructor.
         * @param id [in] a unique id of the extension.
         * @param point [in] the extension point.
         * @param owner [in] the plugin that owns this extension or NULL.
         */
        Extension (const std::string& id, const std::string& point, Plugin* owner);

        /**
         * @copydoc Extension(const std::string&,const std::string&,Plugin*)
         * @param obj [in] a pointer to the object.
         */
        Extension (const std::string& id, const std::string& point, Plugin* owner,
                   rw::core::AnyPtr obj);

        //! @brief Destructor
        virtual ~Extension () {}

        //! @brief a unique id that uniquely identifies this extension in its owner Plugin
        const std::string& getId () { return _desc.id; }

        //! @brief a human readable name of this plugin (may contain spaces)
        const std::string& getName () { return _desc.name; }

        //! @brief a unique global identifier of the extension point that this extension is attached
        //! to
        const std::string& getPoint () const { return _desc.point; }

        //! @copydoc getProperties() const
        rw::core::PropertyMap& getProperties () { return _desc.props; }

#if !defined(SWIG)
        //! @brief the properties/configuration of this extension
        const rw::core::PropertyMap& getProperties () const { return _desc.props; }
#endif

        /**
         * @brief Get the object.
         * @return the object.
         */
        virtual rw::core::AnyPtr getObject () { return _obj; }

        //! @brief get the owner plugin
        Plugin* getOwner () { return _owner; }

      private:
        friend class Plugin;

        //! @brief set owner plugin, NOTE use with care
        void setOwner (Plugin* owner) { _owner = owner; }

      private:
        Descriptor _desc;
        Plugin* _owner;
        rw::core::AnyPtr _obj;
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
