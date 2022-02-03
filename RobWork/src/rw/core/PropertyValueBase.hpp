/******************************************************************************
 * Copyright 2020 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RW_CORE_PROPERTYVALUEBASE_HPP_
#define RW_CORE_PROPERTYVALUEBASE_HPP_

/**
 * @file PropertyValueBase.hpp
 * @copydoc PropertyValueBase
 */

#if !defined(SWIG)
#include <rw/core/Event.hpp>
#include <rw/core/PropertyType.hpp>
#include <rw/core/Ptr.hpp>

#include <functional>
#include <string>
#endif

namespace rw { namespace core {

//! @addtogroup core
#if !defined(SWIG)
    //! @{
#endif

    //! @brief Base class for Property handling
    class PropertyValueBase
    {
      public:
        //! @brief Smart pointer type to this class
        typedef rw::core::Ptr< PropertyValueBase > Ptr;

        /**
         * @brief Constructor.
         */
        PropertyValueBase ();

        /**
         * @brief Constructor
         *
         * @param type [in] type of the property
         */
        PropertyValueBase (const PropertyType& type);

        /**
         * @brief Destroys PropertyValueBase
         */
        virtual ~PropertyValueBase ();

        /**
         * @brief Construct a clone of the property value.
         *
         * @return a clone.
         */
        virtual PropertyValueBase* clone () const = 0;

        /**
         * @brief Returns the PropertyType
         * @return the PropertyType
         */
        const rw::core::PropertyType& getType () const;

#if !defined(SWIGJAVA)
        /**
         * @brief Method signature for a callback function
         */
        typedef std::function< void (PropertyValueBase*) > PropertyListener;

        //! @brief Type for changed property events.
        typedef rw::core::Event< PropertyListener, PropertyValueBase* > ChangedEvent;

        /**
         * @brief get changed event
         *
         * to add listener use:
         * changedEvent().add(...)
         *
         */
        ChangedEvent& changedEvent () { return _changedEvent; }
#endif

      private:
        /**
         * @brief Type of property
         */
        PropertyType _propertyType;
#if !defined(SWIGJAVA)
        //! changed event handler
        ChangedEvent _changedEvent;
#endif

      private:
        PropertyValueBase (const PropertyValueBase&);
        PropertyValueBase& operator= (const PropertyValueBase&);
    };
#if !defined(SWIG)
//! @}
#endif

}}    // namespace rw::core

#endif /* RW_CORE_PROPERTYVALUEBASE_HPP_ */
