/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/geometry/BSphere.hpp>
#include <rw/geometry/Cylinder.hpp>

using namespace rw::math;
using namespace rw::geometry;

TEST(BSphereTest, basics) {
    double height=0.01;
    double radius=0.004;
	Cylinder a(radius,height);
    double axis= sqrt(radius*radius+height/2*height/2);

    BSphere<> sphere = BSphere<>::fitEigen(*(a.getTriMesh()));

    EXPECT_NEAR(sphere.getRadius(),axis,10e-10);
}