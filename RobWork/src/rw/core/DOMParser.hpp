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

#ifndef RW_CORE_DOMPARSER_HPP
#define RW_CORE_DOMPARSER_HPP

#if !defined(SWIG)
#include <rw/core/DOMElem.hpp>
#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/Ptr.hpp>

#include <string>
#include <vector>
#endif

namespace rw { namespace core {

    /** @addtogroup core */
    /*@{*/

    /**
     * @brief interface for parsing documents in a DOM fasion.
     *
     * The factory method in the DOM parser enables extensions to be added by the user through
     * plugins.
     */
    class DOMParser
    {
      protected:
        //! constructor
        DOMParser (){};

      public:
        //! smart pointer type
        typedef rw::core::Ptr< DOMParser > Ptr;

        //! destructor
        virtual ~DOMParser () {}

        // loading and saving
        /**
         * @brief parse from file
         * @param filename [in] name of file
         */
        virtual void load (const std::string& filename) = 0;

        /**
         * @brief parse from stream
         * @param input [in] input stream
         */
        virtual void load (std::istream& input) = 0;

        /**
         * @brief save DOM structure to file
         * @param filename [in] filename
         */
        virtual void save (const std::string& filename) = 0;

        /**
         * @brief save DOM structure to stream
         * @param input
         */
        virtual void save (std::ostream& input) = 0;

        /**
         * @brief specify the schema file
         *
         * @note not all DOMParser implementations support schema validation
         * @param filename
         */
        virtual void setSchema (const std::string& filename) { _schemaFile = filename; }

        /**
         * @brief get the top/root element in the DOM structure
         * @return root element of dom structure
         */
        virtual DOMElem::Ptr getRootElement () = 0;

        /**
         * @brief Enable/disable debugging to the debug Log.
         * @param debug [in] true to enable debug output.
         */
        virtual void setDebug (bool debug) = 0;

        // extra stuff that can be added in top of document
        /**
         * @brief make an instance of the default DOM parser
         * @return a DOM parser for xml files
         */
        static rw::core::Ptr< DOMParser > make ();

#if !defined(SWIG)
        /**
         * @addtogroup extensionpoints
         * @extensionpoint{ rw::core::DOMParser::Factory,rw::core::DOMParser,rw.core.DOMParser }
         * \class DOMParser
         */
#endif
        /**
         * @brief a factory for DOMParsers. This factory defines an
         * extension point for DOMParsers. Typically this is for parsing xml files,
         * however, anything that parses from some stream or file into a DOM structure
         * can be a DOMParser.
         */
        class Factory : public rw::core::ExtensionPoint< DOMParser >
        {
          public:
            //! constructor
            Factory () :
                rw::core::ExtensionPoint< DOMParser > ("rw.core.DOMParser",
                                                       "DOM capable parser of fstream and files"){};

            /**
             * @brief get a DOM parser for a specific file format
             * @param format [in] the extension identifying the file format (without initial dot).
             * @return a parser if found, false otherwise.
             */
            static rw::core::Ptr< DOMParser > getDOMParser (const std::string& format);

            /**
             * @brief check if the factory has a DOM parser for a specific format
             * @param format [in] file ending like: xml, ini, txt...
             * @return
             */
            static bool hasDOMParser (const std::string& format);

            /**
             * @brief get a list of supported formats
             * @return
             */
            static std::vector< std::string > getSupportedFormats ();
        };

      protected:
        //! @brief Filename of schema.
        std::string _schemaFile;
    };

    /*@}*/
}}    // namespace rw::core

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    using namespace rw::core;
}}    // namespace rw::common

#endif
