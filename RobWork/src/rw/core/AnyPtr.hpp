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

#ifndef RW_CORE_ANY_HPP
#define RW_CORE_ANY_HPP

#if !defined(SWIG)
#include <rw/core/Ptr.hpp>

#include <typeinfo>
#endif

namespace rw { namespace core {

    //! @brief Smart pointer that can point to any type, and optionally takes ownership of the
    //! object pointer.
    class AnyPtr
    {
      public:
        //! @brief Construct empty null pointer.
        AnyPtr () : content (NULL) {}

        /**
         * @brief constructor - ownership of pointer is taken
         * @param value [in] a raw pointer.
         */
        template< typename ValueType >
        AnyPtr (ValueType* value) : content (ownedPtr (new holder< ValueType > (value)))
        {}

        /**
         * @brief Construct from Ptr - shares ownership.
         * @param value [in] a smart pointer.
         */
        template< typename ValueType >
        AnyPtr (const rw::core::Ptr< ValueType >& value) :
            content (ownedPtr (new holder< ValueType > (value)))
        {}

        /**
         * @brief Copy constructor - ownership is shared.
         * @param other [in] other AnyPtr object.
         */
        AnyPtr (const AnyPtr& other) : content (other.content ? other.content->clone () : NULL) {}

        //! @brief Destructor.
        ~AnyPtr ()
        {
            if (!content.isShared () && !content.isNull ()) {
                delete content.get ();
            }
        }

        /**
         * @brief Cast to a specific smart pointer type.
         * @return a Ptr object pointing to object if cast success, otherwise a NULL Ptr object is
         * returned.
         */
        template< class S > Ptr< S > cast () const
        {
            rw::core::Ptr< holder< S > > content_cast = content.cast< holder< S > > ();
            if (content_cast.isNull ())
                return Ptr< S > ();
            return content_cast->_ptr;
        }

        /**
         * @brief The pointer stored in the object.
         * @return raw pointer.
         */
        template< class S > S* get ()
        {
            rw::core::Ptr< holder< S > > content_cast = content.cast< holder< S > > ();
            if (content_cast.isNull ())
                return NULL;
            return content_cast->_ptr.get ();
        }

#if !defined(SWIG)
        /**
         * @brief Support for implicit conversion to bool.
         */
        operator void* () const { return content->getVoidPtr (); }
#endif
        /**
         * @brief Equality operator. This only tests if the pointers to the referenced objects are
         * the same and NOT if the smart pointers are the same.
         * @param p [in] smart pointer to compare with
         * @return true if the referenced objects are the same
         */
        template< class A > bool operator== (const Ptr< A >& p) const
        {
            return content->getVoidPtr () == p.get ();
        }
#if !defined(SWIG)
        /**
         * @brief Tests if the smart pointer points to the same instance as \b p
         * @return true if equal, false otherwise.
         */
        bool operator== (void* p) const { return content->getVoidPtr () == p; }
#endif

        /**
         * @brief copy assignemt
         * @param rhs [in] the Ptr to copy.
         */
        AnyPtr& operator= (const AnyPtr& rhs)
        {
            if (!content.isShared () && !content.isNull ()) {
                delete content.get ();
            }
            content = rhs.content;
            return *this;
        }

        /**
         * @brief Check if pointer is null.
         * @return true is the smart pointer is null.
         */
        bool isNull () const
        {
            if (content == NULL) {
                return true;
            }
            return content->getVoidPtr () == NULL;
        }

        /**
         * @brief Get type info for the object pointed to.
         * @return type_info object.
         */
        const std::type_info& type () const { return content->type (); }

      private:    // types
        class placeholder
        {
          public:    // structors
            virtual ~placeholder () {}

          public:    // queries
            virtual const std::type_info& type () const = 0;

            virtual placeholder* clone () const = 0;

            virtual void* getVoidPtr () const = 0;
        };

        template< typename ValueType > class holder : public placeholder
        {
          public:    // structors
            holder (ValueType* value) : _ptr (value) {}
            holder (Ptr< ValueType > value) : _ptr (value) {}

          public:    // queries
            virtual const std::type_info& type () const { return typeid (ValueType); }

            virtual placeholder* clone () const
            {
                return dynamic_cast< placeholder* > (new holder (*this));
            }

            virtual void* getVoidPtr () const { return (void*) _ptr.get (); }

          private:    // intentionally left unimplemented
            holder& operator= (const holder&);

          public:
            Ptr< ValueType > _ptr;
        };

      private:    // representation
        template< typename ValueType > friend ValueType* cast (AnyPtr*);

        rw::core::Ptr< placeholder > content;
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
