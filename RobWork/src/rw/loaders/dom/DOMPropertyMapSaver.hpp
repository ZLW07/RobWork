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

#ifndef RW_LOADERS_DOMPROPERTYMAPSAVER_HPP
#define RW_LOADERS_DOMPROPERTYMAPSAVER_HPP

#include <rw/core/DOMElem.hpp>
#include <rw/core/Ptr.hpp>

namespace rw { namespace core {
    class DOMParser;
    class PropertyBase;
    class PropertyMap;
    class PropertyValueBase;
}} // namespace rw::core

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Class for saving rw::core::PropertyMap to XML
     *
     * The saver is capable of saving all types defined in rw::common::PropertyType.
     *
     * Implemented using RobWork DOM parser abstraction.
     */
    class DOMPropertyMapSaver
    {
      public:
        /**
         * @brief Writes a single property to a DOMElement
         *
         * Constructs a new DOMElement for the document \b parent and writes the property to it.
         *
         * @throws rw::core::Exception if the type of the property is not supported.
         *
         * @param property [in] Property to save
         * @param parent [in] DOMDocument which should contain the property representation
         */
        static void save (rw::core::Ptr< rw::core::PropertyBase > property,
                          rw::core::DOMElem::Ptr parent);

        /**
         * @brief Saves properties of a PropertyMap as childs to \b element.
         *
         * Constructs element representing the properties in \b map and adds these as childs to \b
         * element.
         *
         * Throws rw::core::Expcetion if the type of a property is not supported.
         *
         * @param map [in] Map of properties to save.
         * @param parent [in] DOMDocument which should contain the PropertyMap representation
         */
        static void save (const rw::core::PropertyMap& map, rw::core::DOMElem::Ptr parent);

        /**
         * @brief Saves the properties of \b map to file named \b filename
         *
         * @throws rw::core::Exception if the type of a property is not supported.
         *
         * @param map [in] Map of properties to save
         * @param filename [in] Filename
         */
        static void save (const rw::core::PropertyMap& map, const std::string& filename);

        /**
         * @brief Writes the properties of \b map to \b outstream
         *
         * @throws rw::core::Exception if the type of a property is not supported.
         *
         * @param map [in] Map of properties to save
         * @param outstream [in] Output stream
         */
        static void write (const rw::core::PropertyMap& map, std::ostream& outstream);

        /**
         * @brief Creates DOMDocument for \b map
         *
         * @throws rw::core::Exception if the type of a property is not supported.
         *
         * @param map [in] Map of properties
         * @param parser [in] DOMParser to use
         * @return DOMDocument containing properties.
         */
        static rw::core::DOMElem::Ptr
        createDOMDocument (const rw::core::PropertyMap& map,
                           rw::core::Ptr< rw::core::DOMParser > parser);

        /**
         * @brief Utility class which initializes local static variables.
         *
         * If the DOMPropertyMapSaver is used outside main (as a part of global
         * initialization/destruction), the Initializer should be used explicitly to control the
         * static initialization/destruction order.
         *
         * Notice that the Initializer is automatically defined as a global variable, hence it
         * should not be necessary to specify the initializer explicitly if DOMPropertyMapSaver is
         * to be used in local static initialization/destruction.
         */
        class Initializer
        {
          public:
            //! @brief Initializes when constructed.
            Initializer ();
        };

      private:
        static const Initializer initializer;
        DOMPropertyMapSaver (){}

        static void save (const rw::core::PropertyValueBase& value, rw::core::DOMElem::Ptr parent);
    };

    /** @} */

}}    // namespace rw::loaders

#endif    // end include guard
