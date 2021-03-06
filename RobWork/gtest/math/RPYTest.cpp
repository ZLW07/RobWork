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
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::math;

TEST(RPYTest, Test)
{
    //Test default constructor
    RPY<> rpy0;
    EXPECT_EQ(rpy0[0] , 0.);
    EXPECT_EQ(rpy0[1] , 0.);
    EXPECT_EQ(rpy0[2] , 0.);

    //Test RPY(T alpha, T beta, T gamma) constructor

    RPY<> rpy1(0.1, 0.2, 0.3);
    EXPECT_EQ(rpy1[0] , 0.1);
    EXPECT_EQ(rpy1[1] , 0.2);
    EXPECT_EQ(rpy1[2] , 0.3);

    //Test toRotation3D and RPY(const Rotation3D<T>&) constructor
    Rotation3D<> rot3d = rpy1.toRotation3D();

    RPY<> rpy2(rot3d);
    EXPECT_LT(fabs(rpy2[0] - rpy1[0]) , 1e-16);
    EXPECT_LT(fabs(rpy2[1] - rpy1[1]) , 1e-16);
    EXPECT_LT(fabs(rpy2[2] - rpy1[2]) , 1e-16);

    rpy1[0] = 0;
    rpy1[1] = Pi/2;
    rpy1[2] = 0;

    rot3d = rpy1.toRotation3D();
    EXPECT_EQ(rot3d(0,2) , 1);
    EXPECT_EQ(rot3d(1,1) , 1);
    EXPECT_EQ(rot3d(2,0) , -1);

    RPY<> rpy3(rot3d);
    EXPECT_EQ(rpy3[0] , rpy1[0]);
    EXPECT_EQ(rpy3[1] , rpy1[1]);
    EXPECT_EQ(rpy3[2] , rpy1[2]);

    rpy1[0] = -2.5;
    rpy1[1] = 0.4;
    rpy1[2] = 4.2;
	RPY<float> rpyf;
	rpyf = cast<float>(rpy1);
	for (size_t i = 0; i<3; i++)
		EXPECT_EQ(rpyf[i] , (float)rpy1[i]);
	rpyf = rw::math::cast<float>(rpy1); // qualified lookup
	for (size_t i = 0; i<3; i++)
		EXPECT_EQ(rpyf[i] , (float)rpy1[i]);

    /* Test comparison operators operator== and operator!= */
    const RPY<double> comp1(1.1, -2.2, 3.3);
    const RPY<double> comp2(1.1, -2.2, 3.3);
    EXPECT_EQ(comp1 , comp2);
    EXPECT_TRUE(!(comp1 != comp2));
    const RPY<double> comp3(1.1, 2.2, -3.3);
    EXPECT_NE(comp1 , comp3);
    EXPECT_TRUE(!(comp1 == comp3));
}
