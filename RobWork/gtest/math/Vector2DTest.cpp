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

#include <gtest/gtest.h>
#include <rw/math/Vector2D.hpp>

TEST(Vector2DTest, Vector2DTest_ADL) {
	rw::math::Vector2D<double> vd1(0.4, -0.7);
	rw::math::Vector2D<double> vd2(0.9, 0.1);
	rw::math::Vector2D<> v5(3.0, 4.0);

	// Test Argument-Dependent Lookup (ADL)
	EXPECT_LT(fabs(dot(vd1, vd2) - 0.29) , 1e-15);
	const double cr = cross(rw::math::Vector2D<>(2, 1), rw::math::Vector2D<>(1, 3));
	EXPECT_EQ(cr , 5);
	const rw::math::Vector2D<> v5_norm = normalize(v5);
	EXPECT_LT(fabs(v5_norm.norm2() - 1) , 1e-15);
}

using namespace rw::math;

TEST(Vector2DTest, Vector2D)
{
    Vector2D<> v1(1.0, 2.0);
    Vector2D<> v2(v1);
    Vector2D<> v3 = v1+v2;
    Vector2D<> v4 = Vector2D<>(2.0, 4.0);

    EXPECT_EQ ( (v3-v4).normInf() , 0);

    Vector2D<> v5(3.0, 4.0);
	Vector2D<> v5_norm;
	v5_norm = normalize(v5);
	EXPECT_LT(fabs(v5_norm.norm2() - 1) , 1e-15);
	v5_norm = rw::math::normalize(v5); // qualified lookup
	EXPECT_LT(fabs(v5_norm.norm2() - 1) , 1e-15);
    double l = v5.norm2();
    EXPECT_LT(fabs(v5_norm(0)-v5(0)/l) , 1e-15);
    EXPECT_LT(fabs(v5_norm(1)-v5(1)/l) , 1e-15);

    v5(0) = l;
    v5(1) = 2*l;
    EXPECT_EQ(v5(0) , l);
    EXPECT_EQ(v5(1) , 2*l);

    Vector2D<std::string> vs1("x1", "y1");

    EXPECT_EQ(vs1(0) , "x1");
    EXPECT_EQ(vs1(1) , "y1");

	double cr;
	cr = cross(Vector2D<>(2, 1), Vector2D<>(1, 3));
	EXPECT_EQ(cr , 5);
	cr = rw::math::cross(Vector2D<>(2, 1), Vector2D<>(1, 3)); // qualified
	EXPECT_EQ(cr , 5);

    Vector2D<double> vd(1.51, -2.51);
    Vector2D<int> vi = cast<int>(vd);
    EXPECT_EQ(vi(0) , 1);
    EXPECT_EQ(vi(1) , -2);

    Vector2D<double> vd1(0.4, -0.7);
    Vector2D<double> vd2(0.9, 0.1);
	EXPECT_LT(fabs(dot(vd1, vd2) - 0.29) , 1e-15);
	EXPECT_LT(fabs(rw::math::dot(vd1, vd2) - 0.29) , 1e-15);

    /* Test comparison operators operator== and operator!= */
    const Vector2D<double> comp1(1.1, -2.2);
    const Vector2D<double> comp2(1.1, -2.2);
    EXPECT_EQ(comp1 , comp2);
    EXPECT_TRUE(!(comp1 != comp2));
    const Vector2D<double> comp3(1.1, 2.2);
    EXPECT_NE(comp1 , comp3);
    EXPECT_TRUE(!(comp1 == comp3));
}
