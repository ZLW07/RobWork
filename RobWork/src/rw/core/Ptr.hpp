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

#ifndef RW_CORE_PTR_HPP
#define RW_CORE_PTR_HPP

/**
   @file Ptr.hpp
 */

#ifdef RW_USE_BOOST_PTR
#ifndef RW_USE_BOOST_PTR_COMPLIANCE
#define RW_USE_BOOST_PTR_COMPLIANCE
#endif
#endif

#ifdef RW_USE_BOOST_PTR_COMPLIANCE
#include <boost/shared_ptr.hpp>
#endif

#if !defined(SWIG)
#include <memory>
#include <type_traits>
#include <vector>
#endif

namespace rw { namespace core {

    /** @addtogroup core */
    /*@{*/

    /**
   @brief Ptr stores a pointer and optionally takes ownership of the value.
 */
    template< class T > class Ptr
    {
      public:
#ifdef RW_USE_BOOST_PTR_COMPLIANCE
        //! The type of a Boost shared pointer.
        typedef boost::shared_ptr< T > boost_shared_ptr;
#endif

        //! The type of a standard shared pointer.
        typedef std::shared_ptr< T > cpp_shared_ptr;
#ifdef RW_USE_BOOST_PTR
        //! The internal type of shared pointer used.
        typedef boost_shared_ptr shared_ptr;
#else
        //! The internal type of shared pointer used.
        typedef cpp_shared_ptr shared_ptr;
#endif

        //! Pointer type
        typedef T* pointer;

        //! Reference type
        typedef T& reference;

        //! Value type
        typedef T value_type;

        /**
         * @brief Default constructor yielding a NULL-pointer.
         */
        Ptr () : _ptr (0), _owned_ptr () {}

        /**
         * @brief Do not take ownership of \b ptr.
         *
         * \b ptr can be null.
         *
         * The constructor is implicit on purpose.
         */
        Ptr (T* ptr) : _ptr (ptr), _owned_ptr () {}

#ifdef RW_USE_BOOST_PTR_COMPLIANCE
        /**
   @brief Take ownership of \b ptr.

   \b ptr can be null.

   The constructor is implicit on purpose.
     */
        Ptr (boost_shared_ptr ptr) :
            _ptr (ptr.get ()),
#ifdef RW_USE_BOOST_PTR
            _owned_ptr (ptr)
#else
            _owned_ptr (ptr.get (), [ptr] (T*) {})
#endif
        {}
#endif

#if !defined(SWIG)
        /**
         * @brief Take ownership of \b ptr.
         *
         * \b ptr can be null.
         *
         * The constructor is implicit on purpose.
         */
        Ptr (cpp_shared_ptr ptr) :
            _ptr (ptr.get ()),
#ifdef RW_USE_BOOST_PTR
            _owned_ptr (ptr.get (), [ptr] (T*) {})
#else
            _owned_ptr (ptr)
#endif
        {}
#endif

        /**
         * @brief destructor
         */
        ~Ptr () {}

        /**
         * @brief Cast the smart pointer to a different type.
         * @return smart pointer that can be null if cast was not possible.
         */
        template< class S > Ptr< S > cast ()
        {
            if (_owned_ptr)
#ifdef RW_USE_BOOST_PTR
                return Ptr< S > (boost::dynamic_pointer_cast< S > (_owned_ptr));
#else
                return Ptr< S > (std::dynamic_pointer_cast< S > (_owned_ptr));
#endif
            else
                return Ptr< S > (dynamic_cast< S* > (_ptr));
        }

        //! @copydoc cast()
        template< class S > Ptr< S > cast () const
        {
            if (_owned_ptr)
#ifdef RW_USE_BOOST_PTR
                return Ptr< S > (boost::dynamic_pointer_cast< S > (_owned_ptr));
#else
                return Ptr< S > (std::dynamic_pointer_cast< S > (_owned_ptr));
#endif
            else
                return Ptr< S > (dynamic_cast< S* > (_ptr));
        }

        /**
         * @brief Cast the smart pointer statically to a different type.
         *
         * This is more efficient if it is already known that the object is of a certain type.
         * If this is not known, please use the more safe cast() instead.
         *
         * @return smart pointer that can be null if cast was not possible.
         */
        template< class S > Ptr< S > scast ()
        {
            if (_owned_ptr)
#ifdef RW_USE_BOOST_PTR
                return Ptr< S > (boost::static_pointer_cast< S > (_owned_ptr));
#else
                return Ptr< S > (std::static_pointer_cast< S > (_owned_ptr));
#endif
            else
                return Ptr< S > (static_cast< S* > (_ptr));
        }

        //! @copydoc scast()
        template< class S > Ptr< S > scast () const
        {
            if (_owned_ptr)
#ifdef RW_USE_BOOST_PTR
                return Ptr< S > (boost::static_pointer_cast< S > (_owned_ptr));
#else
                return Ptr< S > (std::static_pointer_cast< S > (_owned_ptr));
#endif
            else
                return Ptr< S > (static_cast< S* > (_ptr));
        }

#if !defined(SWIG)

        /**
         * @brief Construct smart pointer from other smart pointer.
         *
         * @param p the other (compatible) smart pointer.
         */
        template< class S >
        Ptr< T > (Ptr< S > const& p,
                  typename std::enable_if< std::is_base_of< T, S >::value >::type* = 0)
        {
            _owned_ptr = p.getSharedPtr ();
            _ptr       = p.get ();
        }
#else
        /**
         * @brief Construct smart pointer from other smart pointer.
         *
         * @param p the other (compatible) smart pointer.
         */
        template< class S > Ptr (const Ptr< S >& p)
        {
            _owned_ptr = p.getSharedPtr ();
            _ptr       = p.get ();
        }
#endif

        /**
         * @brief The pointer stored in the object.
         */
        pointer get () const { return _ptr; }

        /**
         * @brief Dereferencing operator.
         */
        reference operator* () const { return *get (); }

        /**
         * @brief Member access operator.
         */
        pointer operator-> () const { return get (); }

#if !defined(SWIG)
        /**
         *@brief Support for implicit conversion to bool.
         */
        operator const void* () const { return get (); }
#endif

#ifdef RW_USE_BOOST_PTR_COMPLIANCE
        Ptr< T >& operator= (boost_shared_ptr& ptr)
        {
            _ptr       = ptr.get ();
            _owned_ptr = ptr;
            return *this;
        }
#endif
#if !defined(SWIG)
        Ptr< T >& operator= (cpp_shared_ptr& ptr)
        {
            _ptr       = ptr.get ();
            _owned_ptr = ptr;
            return *this;
        }
#endif

        /**
         * @brief equallity operator, this only tests if the pointers to the referenced objects are
         * the same and NOT if the smart pointers are the same.
         * @param p [in] smart pointer to compare with
         * @return true if the referenced objects are the same
         */
        template< class A > bool operator== (const Ptr< A >& p) const { return get () == p.get (); }

#if !defined(SWIG)
        /**
         * @brief Tests if the smart pointer points to the same instance as \b p
         */
        bool operator== (void* p) const { return get () == p; }

        /**
         * @brief Tests if the smart pointer points to different from the instance of \b p
         */
        bool operator!= (void* p) const { return get () != p; }
#else 
        PTR_EQ_C_PTR;
#endif

        /**
         * @brief check if this Ptr has shared ownership or none
         * ownership
         * @return true if Ptr has shared ownership, false if it has no ownership.
         */
        bool isShared () const
        {
            if (_owned_ptr)
                return true;
            else
                return false;
        }

        /**
         * @brief checks if the pointer is null
         * @return Returns true if the pointer is null
         */
        bool isNull () const { return get () == NULL; }

#ifdef RW_USE_BOOST_PTR_COMPLIANCE
        /**
         * @brief Returns the shared pointer used internally
         */
        boost_shared_ptr getBoostSharedPtr () const
        {
#ifdef RW_USE_BOOST_PTR
            return _owned_ptr;
#else
            cpp_shared_ptr ptr_cp = _owned_ptr;
            return boost_shared_ptr (_owned_ptr.get (), [ptr_cp] (T*) {});
#endif
        }
#endif
#if !defined(SWIG)
        /**
         * @brief Returns the shared pointer used internally
         */
        cpp_shared_ptr getCppSharedPtr () const
        {
#ifdef RW_USE_BOOST_PTR
            boost_shared_ptr ptr_cp = _owned_ptr;
            return cpp_shared_ptr (_owned_ptr.get (), [ptr_cp] (T*) {});
#else
            return _owned_ptr;
#endif
        }
#endif
#if !defined(SWIG)
#ifdef RW_USE_BOOST_PTR_COMPLIANCE
        /**
         * @brief Implicitly convert to boost::shared_ptr type.
         *
         * @return boost::shared_ptr
         */
        operator boost_shared_ptr () const { return getBoostSharedPtr (); }
#endif
        /**
         * @brief Implicitly convert to std::shared_ptr type.
         *
         * @return std::shared_ptr
         */
        operator cpp_shared_ptr () const { return getCppSharedPtr (); }

        /**
         * @brief Returns the shared pointer used internally
         */
        shared_ptr getSharedPtr () const { return _owned_ptr; }
#endif
#if !defined(SWIG)
        /**
         * @brief Get const Pointer.
         * @return a copy in the form of Ptr<const T>;
         */
        Ptr< const T > cptr () const
        {
            if (this->isShared ()) {
                return Ptr< const T > (
                    std::const_pointer_cast< const T > (this->getCppSharedPtr ()));
            }
            else {
                return this->scast< const T > ();
            }
        }
#endif
      private:
        T* _ptr;
        shared_ptr _owned_ptr;
    };

#if !defined(SWIGLUA)
    /**
     * @brief Comparator for comparing an ordinary pointer with a smart pointer
     *
     * @note If comparing two instances of a class without specifying the equal operator
     * this method might be called.
     */
    template< class T, class R > bool operator== (void* p, const Ptr< R >& g)
    {
        return p == g.get ();
    }
#endif

    /**
     * @brief A Ptr that takes ownership over a raw pointer \b ptr.
     * @relates Ptr
     */
    template< class T > Ptr< T > ownedPtr (T* ptr)
    {
        return Ptr< T > (typename Ptr< T >::shared_ptr (ptr));
    }

    /**
     * @brief Convert a vector of Ptr to a vector of std::shared_ptr.
     *
     * @param ptrs [in] vector of Ptr.
     * @return a vector of std::shared_ptr with same size.
     * @relates Ptr
     */
    template< class T >
    std::vector< std::shared_ptr< T > > toStd (const std::vector< Ptr< T > >& ptrs)
    {
        std::vector< std::shared_ptr< T > > res (ptrs.size ());
        for (std::size_t i = 0; i < ptrs.size (); i++)
            res[i] = ptrs[i];
        return res;
    }

    /**
     * @brief Convert a vector of std::shared_ptr to a vector of Ptr.
     *
     * @param ptrs [in] vector of std::shared_ptr.
     * @return a vector of Ptr with same size.
     * @relates Ptr
     */
    template< class T >
    std::vector< Ptr< T > > fromStd (const std::vector< std::shared_ptr< T > >& ptrs)
    {
        std::vector< Ptr< T > > res (ptrs.size ());
        for (std::size_t i = 0; i < ptrs.size (); i++)
            res[i] = ptrs[i];
        return res;
    }

#ifdef RW_USE_BOOST_PTR_COMPLIANCE
    /**
     * @brief Convert a vector of Ptr to a vector of boost::shared_ptr.
     *
     * @param ptrs [in] vector of Ptr.
     * @return a vector of boost::shared_ptr with same size.
     * @relates Ptr
     */
    template< class T >
    std::vector< boost::shared_ptr< T > > toBoost (const std::vector< Ptr< T > >& ptrs)
    {
        std::vector< boost::shared_ptr< T > > res (ptrs.size ());
        for (std::size_t i = 0; i < ptrs.size (); i++)
            res[i] = ptrs[i];
        return res;
    }

    /**
     * @brief Convert a vector of boost::shared_ptr to a vector of Ptr.
     *
     * @param ptrs [in] vector of boost::shared_ptr.
     * @return a vector of Ptr with same size.
     * @relates Ptr
     */
    template< class T >
    std::vector< Ptr< T > > fromBoost (const std::vector< boost::shared_ptr< T > >& ptrs)
    {
        std::vector< Ptr< T > > res (ptrs.size ());
        for (std::size_t i = 0; i < ptrs.size (); i++)
            res[i] = ptrs[i];
        return res;
    }
#endif

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
