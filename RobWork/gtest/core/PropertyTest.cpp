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

#include <gtest/gtest.h>

#include <rw/core/Property.hpp>
#include <rw/core/PropertyType.hpp>
#include <rw/core/PropertyValueBase.hpp>

#include <functional>

using namespace rw::core;

namespace {
    void listen(int* cnt, PropertyBase* const pbase) {
        (*cnt)++;
    }
    void listenValue(int* cnt, PropertyValueBase* const pbase) {
        (*cnt)++;
    }
}

TEST(Property, double) {
    const double val = 0.1;
    // Test ordinary constructor, clone() and standard interface functions
    {
        Property<double> property("TestId", "Desc", val);
        EXPECT_EQ("TestId", property.getIdentifier());
        EXPECT_EQ("Desc", property.getDescription());
        EXPECT_EQ(0.1, property.getValue());
        EXPECT_EQ(PropertyType::Types::Double, property.getType().getId());
        EXPECT_EQ(0.1, property.getPropertyValue().getValue());
        EXPECT_EQ(PropertyType::Types::Double, property.getPropertyValue().getType().getId());
        const Property<double>* const clone = property.clone();
        EXPECT_EQ("TestId", clone->getIdentifier());
        EXPECT_EQ("Desc", clone->getDescription());
        EXPECT_EQ(0.1, clone->getValue());
        EXPECT_EQ(PropertyType::Types::Double, clone->getType().getId());
        EXPECT_EQ(0.1, clone->getPropertyValue().getValue());
        EXPECT_EQ(PropertyType::Types::Double, clone->getPropertyValue().getType().getId());
        property.setDescription("NewDesc");
        property.setValue(0.2);
        EXPECT_EQ("NewDesc", property.getDescription());
        EXPECT_EQ(0.2, property.getValue());
        EXPECT_EQ(0.2, property.getPropertyValue().getValue());
        EXPECT_EQ("Desc", clone->getDescription());
        EXPECT_EQ(0.1, clone->getValue());
        EXPECT_EQ(0.1, clone->getPropertyValue().getValue());
    }
    // Test constructor with explicit type
    {
        Property<double> property("TestId", "Desc", PropertyType::Types::String, val);
        EXPECT_EQ("TestId", property.getIdentifier());
        EXPECT_EQ("Desc", property.getDescription());
        EXPECT_EQ(0.1, property.getValue());
        EXPECT_EQ(PropertyType::Types::String, property.getType().getId());
    }
    // Test changedEvent()
    {
        using std::placeholders::_1;
        Property<double> property("TestId", "Desc", val);
        int cnt1 = 0;
        int cnt2 = 0;
        int cnt3 = 0;
        property.changedEvent().add(std::bind(&listen, &cnt1, _1));
        property.changedEvent().add(std::bind(&listen, &cnt2, _1));
        property.getPropertyValue().changedEvent().add(std::bind(&listenValue, &cnt3, _1));
        EXPECT_EQ(0, cnt1);
        EXPECT_EQ(0, cnt2);
        EXPECT_EQ(0, cnt3);
        property.setValue(val);
        EXPECT_EQ(1, cnt1);
        EXPECT_EQ(1, cnt2);
        EXPECT_EQ(1, cnt3);
        property.getValue() = 0.3;
        EXPECT_EQ(0.3, property.getValue());
        EXPECT_EQ(1, cnt1);
        EXPECT_EQ(1, cnt2);
        EXPECT_EQ(1, cnt3);
        property.getPropertyValue().setValue(0.4);
        EXPECT_EQ(0.4, property.getValue());
        EXPECT_EQ(2, cnt1);
        EXPECT_EQ(2, cnt2);
        EXPECT_EQ(2, cnt3);
        property.getPropertyValue().getValue() = 0.5;
        EXPECT_EQ(0.5, property.getValue());
        EXPECT_EQ(2, cnt1);
        EXPECT_EQ(2, cnt2);
        EXPECT_EQ(2, cnt3);
        property.setDescription("NewDesc");
        EXPECT_EQ(3, cnt1);
        EXPECT_EQ(3, cnt2);
        EXPECT_EQ(2, cnt3);
    }
    // Test rw::core::toProperty() function
    {
        PropertyBase::Ptr pbase = ownedPtr(new Property<double>("TestId", "Desc", val));
        EXPECT_EQ(nullptr, toProperty<float>(pbase));
        Property<double>::Ptr property = toProperty<double>(pbase);
        EXPECT_EQ(0.1, property->getValue());
    }
}
