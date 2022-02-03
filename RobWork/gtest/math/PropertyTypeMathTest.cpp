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
#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Transform2D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

using rw::core::PropertyType;
using namespace rw::math;

TEST(PropertyTypeMath, getType) {
    {
        Vector3D<> value;
        EXPECT_EQ(PropertyType::Types::Vector3D, PropertyType::getType(value).getId());
    }
    {
        Vector2D<> value;
        EXPECT_EQ(PropertyType::Types::Vector2D, PropertyType::getType(value).getId());
    }
    {
        Q value;
        EXPECT_EQ(PropertyType::Types::Q, PropertyType::getType(value).getId());
    }
    {
        Transform3D<> value;
        EXPECT_EQ(PropertyType::Types::Transform3D, PropertyType::getType(value).getId());
    }
    {
        Rotation3D<> value;
        EXPECT_EQ(PropertyType::Types::Rotation3D, PropertyType::getType(value).getId());
    }
    {
        Rotation2D<> value;
        EXPECT_EQ(PropertyType::Types::Rotation2D, PropertyType::getType(value).getId());
    }
    {
        RPY<> value;
        EXPECT_EQ(PropertyType::Types::RPY, PropertyType::getType(value).getId());
    }
    {
        EAA<> value;
        EXPECT_EQ(PropertyType::Types::EAA, PropertyType::getType(value).getId());
    }
    {
        Quaternion<> value;
        EXPECT_EQ(PropertyType::Types::Quaternion, PropertyType::getType(value).getId());
    }
    {
        VelocityScrew6D<> value;
        EXPECT_EQ(PropertyType::Types::VelocityScrew6D, PropertyType::getType(value).getId());
    }
}
