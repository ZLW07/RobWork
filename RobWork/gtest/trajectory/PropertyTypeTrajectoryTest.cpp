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

#include <rw/core/PropertyType.hpp>
#include <rw/trajectory/Path.hpp>

using rw::core::PropertyType;
using namespace rw::trajectory;

TEST(PropertyTypeTrajectory, getType) {
    {
        QPath value;
        EXPECT_EQ(PropertyType::Types::QPath, PropertyType::getType(value).getId());
    }
    {
        QPath::Ptr value;
        EXPECT_EQ(PropertyType::Types::QPathPtr, PropertyType::getType(value).getId());
    }
    {
        Transform3DPath value;
        EXPECT_EQ(PropertyType::Types::Transform3DPath, PropertyType::getType(value).getId());
    }
    {
        Transform3DPath::Ptr value;
        EXPECT_EQ(PropertyType::Types::Transform3DPathPtr, PropertyType::getType(value).getId());
    }
}
