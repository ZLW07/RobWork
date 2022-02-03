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

#include <rw/core/PropertyValue.hpp>
#include <rw/core/PropertyType.hpp>

using namespace rw::core;

namespace {
    void listen(int* cnt, PropertyValueBase* const pvalue) {
        (*cnt)++;
    }
}

TEST(PropertyValue, float) {
    const float val = 0.4f;
    // Test ordinary constructor, clone() and standard interface functions
    {
        PropertyValue<float> pval(val);
        EXPECT_EQ(PropertyType::Types::Float, pval.getType().getId());
        EXPECT_EQ(0.4f, pval.getValue());
        const PropertyValue<float>* const clone = pval.clone();
        EXPECT_EQ(PropertyType::Types::Float, clone->getType().getId());
        EXPECT_EQ(0.4f, clone->getValue());
        pval.setValue(0.1f);
        EXPECT_EQ(0.1f, pval.getValue());
        EXPECT_EQ(0.4f, clone->getValue());
    }
    // Test constructor with explicit type
    {
        PropertyValue<float> pval(PropertyType::Types::Double, val);
        EXPECT_EQ(PropertyType::Types::Double, pval.getType().getId());
        EXPECT_EQ(0.4f, pval.getValue());
        const PropertyValue<float>* const clone = pval.clone();
        EXPECT_EQ(PropertyType::Types::Double, clone->getType().getId());
        EXPECT_EQ(0.4f, clone->getValue());
        pval.setValue(0.1f);
        EXPECT_EQ(0.1f, pval.getValue());
        EXPECT_EQ(0.4f, clone->getValue());
    }
    // Test changedEvent()
    {
        using std::placeholders::_1;
        PropertyValue<float> property(val);
        int cnt1 = 0;
        int cnt2 = 0;
        property.changedEvent().add(std::bind(&listen, &cnt1, _1));
        property.changedEvent().add(std::bind(&listen, &cnt2, _1));
        EXPECT_EQ(0, cnt1);
        EXPECT_EQ(0, cnt2);
        property.setValue(val);
        EXPECT_EQ(1, cnt1);
        EXPECT_EQ(1, cnt2);
        property.getValue() = 0.3f;
        EXPECT_EQ(0.3f, property.getValue());
        EXPECT_EQ(1, cnt1);
        EXPECT_EQ(1, cnt2);
    }
}
