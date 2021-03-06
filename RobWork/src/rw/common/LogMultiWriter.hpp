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

#ifndef RW_COMMON_LOGMULTIWRITER_HPP
#define RW_COMMON_LOGMULTIWRITER_HPP

#if !defined(SWIG)
#include <rw/core/LogWriter.hpp>

#include <vector>
#endif
namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Writes log output to multiple LogWriters
     */
    class LogMultiWriter : public rw::core::LogWriter
    {
      public:
        /**
         * @brief Constructs empty LogMultiWriter
         */
        LogMultiWriter ();

        /**
         * @brief Destructor
         */
        ~LogMultiWriter ();

        /**
         * @brief Adds a LogWriter to be written to
         */
        void addWriter (rw::core::LogWriter::Ptr writer);

      protected:
        /**
         * @copydoc LogWriter::write(const std::string&)
         */
        void doWrite (const std::string& str);

        /**
         * @brief Calls flush on the individual writers
         */
        void doFlush ();

        /**
         * @copydoc LogWriter::setTabLevel(int)
         */
        void doSetTabLevel (int tabLevel);

      private:
        std::vector< rw::core::LogWriter::Ptr > _writers;
    };

    /*@}*/
}}    // namespace rw::common

#endif    // end include guard
