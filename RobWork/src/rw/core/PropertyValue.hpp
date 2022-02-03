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

#ifndef RW_CORE_PROPERTYVALUE_HPP_
#define RW_CORE_PROPERTYVALUE_HPP_

/**
 * @file PropertyValue.hpp
 * @copydoc PropertyValue
 */

#if !defined(SWIG)
#include "PropertyValueBase.hpp"
#endif

namespace rw { namespace core {

    //! @addtogroup core
    //! @{

    /**
     * @brief PropertyValue class
     *
     * The PropertyValue class is a template to support property values of any
     * type.
     */
    template<class T>
    class PropertyValue: public PropertyValueBase
    {
        public:
            //! @brief Smart pointer type to this class
            typedef rw::core::Ptr<PropertyValue> Ptr;

            /**
             * @brief Constructs PropertyValue.
             *
             * Constructs a PropertyValue and tries to auto detect the type.
             *
             * @param value [in] value
             */
            PropertyValue(T value):
                PropertyValueBase(PropertyType::getType(value)),
                _value(value)
            {
            }

            /**
             * @brief Constructs PropertyValue.
             * @param type [in] type of property
             * @param value [in] value
             */
            PropertyValue(const PropertyType& type, T value):
                PropertyValueBase(type),
                _value(value)
            {
            }

            /**
             * @brief Destroys PropertyValue
             * If the property value is a pointer, the object pointed to will NOT be destroyed.
             */
            virtual ~PropertyValue()
            {
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
            T& getValue() {
                return _value;
            }

            /**
             * @brief Returns a constant reference to the property value.
             * @return constant reference to the property value.
             */
            const T& getValue() const {
                return _value;
            }

            /**
             * @brief Sets the property value.
             *
             * This function will fire the changed event.
             *
             * @param value [in] the new value of the Property
             */
            void setValue(const T& value)
            {
                _value = value;
#if !defined(SWIGJAVA)
                this->changedEvent().fire(this);
#endif
            }

            //! @copydoc PropertyBase::clone
            PropertyValue<T>* clone() const
            {
                return new PropertyValue<T>(this->getType(), this->_value);
            }

        private:
            T _value;
    };

    //! @}

}} // end namespaces

#endif /* RW_CORE_PROPERTYVALUE_HPP_ */
