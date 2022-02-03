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

#include <rw/core/PropertyMap.hpp>
#include <rw/core/PropertyType.hpp>

using namespace rw::core;

namespace {
    class CustomType {
    };
}

TEST(PropertyTypeCore, PropertyType) {
    {
        PropertyType proptype;
        EXPECT_EQ(PropertyType::Types::Unknown, proptype.getId());
    }
    {
        PropertyType proptype(PropertyType::Types::Int);
        EXPECT_EQ(PropertyType::Types::Int, proptype.getId());
    }
}

TEST(PropertyTypeCore, getType) {
    {
        PropertyMap map;
        EXPECT_EQ(PropertyType::Types::PropertyMap, PropertyType::getType(map).getId());
    }
    {
        PropertyMap::Ptr map;
        EXPECT_EQ(PropertyType::Types::PropertyMapPtr, PropertyType::getType(map).getId());
    }
    {
        std::vector<rw::core::Ptr<rw::core::PropertyValueBase> > list;
        EXPECT_EQ(PropertyType::Types::PropertyValueBasePtrList, PropertyType::getType(list).getId());
    }
    {
        std::string value;
        EXPECT_EQ(PropertyType::Types::String, PropertyType::getType(value).getId());
    }
    {
        float value = 0.0f;
        EXPECT_EQ(PropertyType::Types::Float, PropertyType::getType(value).getId());
    }
    {
        double value = 0.0;
        EXPECT_EQ(PropertyType::Types::Double, PropertyType::getType(value).getId());
    }
    {
        int value = 0;
        EXPECT_EQ(PropertyType::Types::Int, PropertyType::getType(value).getId());
    }
    {
        bool value = true;
        EXPECT_EQ(PropertyType::Types::Bool, PropertyType::getType(value).getId());
    }
    {
        std::vector<std::string> list;
        EXPECT_EQ(PropertyType::Types::StringList, PropertyType::getType(list).getId());
    }
    {
        std::vector<int> list;
        EXPECT_EQ(PropertyType::Types::IntList, PropertyType::getType(list).getId());
    }
    {
        std::vector<double> list;
        EXPECT_EQ(PropertyType::Types::DoubleList, PropertyType::getType(list).getId());
    }
    {
        CustomType t;
        EXPECT_EQ(PropertyType::Types::Unknown, PropertyType::getType(t).getId());
    }
}
