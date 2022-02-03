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

#ifndef RW_LOADERS_DOMPROPERTYMAPLOADER_HPP
#define RW_LOADERS_DOMPROPERTYMAPLOADER_HPP

#include <rw/core/PropertyBase.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/PropertyValueBase.hpp>
#include <rw/core/Ptr.hpp>

namespace rw { namespace core {
    class DOMElem;
}}    // namespace rw::core

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Class for loading rw::core::PropertyMap from XML
     *
     * The loader is capable of loading all type defined in rw::common::PropertyType.
     *
     * Implemented using RobWork DOM parser abstraction.
     */
    class DOMPropertyMapLoader
    {
      public:

        /**
         * @brief Reads in a PropertyValue from DOMElement.
         *
         * May throw rw::core::Exception
         *
         * @param element [in] DOMElement describing Property
         * @return Pointer to the property value.
         */
        static rw::core::PropertyValueBase::Ptr readPropertyValue (
                rw::core::Ptr< rw::core::DOMElem > element);

        /**
         * @brief Reads in a Property from DOMElement.
         *
         * May throw rw::core::Exception
         *
         * @param element [in] DOMElement describing Property
         * @param checkHeader [in] True to check that the header of \b element matches
         * XMLPropertyFormat::PropertyId
         * @return Pointer to the property
         */
        static rw::core::PropertyBase::Ptr readProperty (rw::core::Ptr< rw::core::DOMElem > element,
                                                         bool checkHeader = true);

        /**
         * @brief Reads in a PropertyMap from DOMElement
         *
         * May throw rw::core::Exception
         *
         * @param element [in] DOMElement describing PropertyMap
         * @param checkHeader [in] True to check that the header of \b element matches
         * XMLPropertyFormat::PropertyMapId
         * @return Loaded PropertyMap
         */
        static rw::core::PropertyMap readProperties (rw::core::Ptr< rw::core::DOMElem > element,
                                                     bool checkHeader = true);
        static bool hasProperties (rw::core::Ptr< rw::core::DOMElem > element);

        /**
         * @brief Read in rw::core::PropertyMap from file
         *
         * Throws rw::core::Exception if an error occurs
         *
         * @param filename [in] File to load
         * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema
         * specified in the XML-file if available.
         * @return Loaded PropertyMap
         */
        static rw::core::PropertyMap load (const std::string& filename,
                                           const std::string& schemaFileName = "");

        /**
         * @brief Read in rw::core::PropertyMap from istream
         *
         * Throws rw::core::Exception if an error occurs
         *
         * @param instream [in] Input stream to read from
         * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema
         * specified in the XML-file if available.
         * @return Loaded PropertyMap
         */
        static rw::core::PropertyMap load (std::istream& instream,
                                           const std::string& schemaFileName = "");

        /**
         * @brief Utility class which initializes local static variables.
         *
         * If the DOMPropertyMapLoader is used outside main (as a part of global
         * initialization/destruction), the Initializer should be used explicitly to control the
         * static initialization/destruction order.
         *
         * Notice that the Initializer is automatically defined as a global variable, hence it
         * should not be necessary to specify the initializer explicitly if DOMPropertyMapLoader is
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

        DOMPropertyMapLoader ();
        virtual ~DOMPropertyMapLoader ();
    };

    /** @} */

}}    // namespace rw::loaders

#endif    // end include guard
