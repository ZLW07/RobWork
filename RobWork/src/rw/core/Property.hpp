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

#ifndef RW_CORE_PROPERTY_HPP
#define RW_CORE_PROPERTY_HPP

/**
 * @file Property.hpp
 */
#if !defined(SWIG)
#include <rw/core/PropertyBase.hpp>
#include <rw/core/PropertyValue.hpp>
#include <functional>
#endif
namespace rw { namespace core {

    /** @addtogroup core */
    /*@{*/

    /**
     * @brief Property class
     * The Property class is a template to support properties of any type. A Property
     * is characterized by a string identifier, string description and a value of the
     * template specified type.
     */
    template< class T > class Property : public PropertyBase
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< Property > Ptr;

        /**
         * @brief Constructs Property.
         *
         * Constructs a Property and tries to auto detect the type.
         *
         * @param identifier [in] identifier
         * @param description [in] description
         * @param value [in] value
         */
        Property (const std::string& identifier, const std::string& description, T value) :
            PropertyBase (identifier, description, PropertyType::getType (value)), _value (value)
        {
#if !defined(SWIGJAVA)
            _value.changedEvent().add(std::bind(&Property::valueChanged, this, std::placeholders::_1), this);
#endif
        }

        /**
         * @brief Constructs Property.
         * @param identifier [in] identifier
         * @param description [in] description
         * @param type [in] type of property
         * @param value [in] value
         */
        Property (const std::string& identifier, const std::string& description,
                  const PropertyType& type, T value) :
            PropertyBase (identifier, description, type),
            _value (value)
        {
#if !defined(SWIGJAVA)
            _value.changedEvent().add(std::bind(&Property::valueChanged, this, std::placeholders::_1), this);
#endif
        }

        /**
         * @brief Destroys Property
         * If the property value is a pointer, the object pointed to will NOT be destroyed.
         */
        virtual ~Property ()
        {
#if !defined(SWIGJAVA)
            _value.changedEvent().remove(this);
#endif
        }

        /**
         * @brief Returns a reference to the property value.
         *
         * @note Changing the value returned by reference will NOT fire the
         * changed event. Please consider using the setValue function if
         * possible, or fire the event manually on change, by calling
         * changedEvent().fire()
         *
         * @return reference to the property value.
         */
        T& getValue () { return _value.getValue(); }

        /**
         * @brief Returns a constant reference to the property value.
         * @return constant reference to the property value.
         */
        const T& getValue () const { return _value.getValue(); }

        /**
         * @brief returns reference to the property value
         * @return value
         */
        PropertyValue<T>& getPropertyValue () { return _value; }

        /**
         * @brief returns const reference to the property value
         * @return value
         */
        const PropertyValue<T>& getPropertyValue () const { return _value; }

        /**
         * @brief Sets the property value.
         *
         * This function will fire the changed event.
         *
         * @param value [in] the new value of the Property
         */
        void setValue (const T& value) { _value.setValue(value); }

        /**
           @copydoc PropertyBase::clone
        */
        Property< T >* clone () const
        {
            return new Property< T > (
                this->getIdentifier (), this->getDescription (), this->getType (), this->_value.getValue());
        }

      private:
          void valueChanged(PropertyValueBase* pbase)
          {
              this->changedEvent().fire(this);
          }

          PropertyValue<T> _value;
    };

    /**
     * @brief cast a property base to a specific property. Notice that the pointer
     * returned is owned by the PropertyBase::Ptr.
     * @param base [in] property base pointer
     * @return property of type \b T or null if property is of another type
     */
    template< class T > Property< T >* toProperty (PropertyBase::Ptr base)
    {
        Property< T >* p = dynamic_cast< Property< T >* > (base.get ());
        return p;
    }

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
