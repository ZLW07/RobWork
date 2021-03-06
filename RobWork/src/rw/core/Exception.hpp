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

#ifndef RW_CORE_EXCEPTION_HPP
#define RW_CORE_EXCEPTION_HPP

/**
 * @file Exception.hpp
 */

#if !defined(SWIG)
#include <rw/core/Message.hpp>

#include <exception>
#include <iosfwd>
#include <string>
#endif

namespace rw { namespace core {

    /** @addtogroup core */
    /*@{*/

    /**
     * @brief Standard exception type of RobWork.
     *
     * All exception thrown within RobWork are of the type Exception.
     *
     * An exception contains a message (of type Message) for the user and
     * nothing else.
     */
    class Exception : public std::exception
    {
      public:
        /**
         * @brief This constructor creates an empty Exception and should not be used
         */
        Exception () : _id (-1), _message ("unknown", -1) {}

        /**
         * @brief Constructor
         *
         * @param message [in] A message for a user.
         */
        Exception (const rw::core::Message& message); 

        /**
         * @brief Constructor
         *
         * @param id [in] Integer Id to identify the exception
         * @param message [in] A message for a user.
         */
        Exception (int id, const rw::core::Message& message);

        virtual ~Exception () throw (){};

        /**
         * @brief The message for the user describing the reason for the error.
         *
         * @return  The message for the user.
         */
        const rw::core::Message& getMessage () const { return _message; }

        /**
         * @brief get id of this exception message
         * @return id
         */
        int getId () const { return _id; }

        /**
         * @brief readable description of this esception
         * @return string description
         */
        // std::string what() const {
        //    return _whatMsg;
        //}

        /**
         * @brief readable description of this esception
         * @return string description
         */
        const char* what () const throw () { return _whatMsg.c_str (); }

#if !defined(SWIG)
        /**
         * @brief Format to \b out the message of the exception \b exp.
         *
         * The format for the text is
         *	\code
         * 	<file>:<line> <message>
         *	\endcode
         * @return The stream \b out.
         */
        friend std::ostream& operator<< (std::ostream& out, const Exception& exp)
        {
            out << exp.getMessage ();
            return out;
        }
#else
        TOSTRING (rw::core::Exception);
#endif

      private:
        int _id;
        rw::core::Message _message;
        std::string _whatMsg;
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

#endif    // end include guard
