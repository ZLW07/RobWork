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


#include <iostream>

#include <gtest/gtest.h>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>

TEST(VelocetyScrew6DTest, VelocityScrew6DTest_ADL) {
	rw::math::Vector3D<> linear1(1, 2, 3);
	rw::math::EAA<> angular1(4, 5, 6);
	rw::math::VelocityScrew6D<> screw(linear1, angular1);

	// Test Argument-Dependent Lookup (ADL)
	EXPECT_LT(fabs(norm1(screw) - (1 + 2 + 3 + 4 + 5 + 6)) , 1e-15);
	EXPECT_LT(fabs(norm2(screw) - sqrt(1.0 * 1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5 + 6 * 6)) , 1e-15);
	EXPECT_LT(fabs(normInf(screw) - 6) , 1e-15);
}

using namespace rw::math;

TEST (VelocityScrew6DTest, VelocetySrew6D) {
  {
    Transform3D<> T = Transform3D<>::identity();
    VelocityScrew6D<> screw(T);
    for (size_t i = 0; i<6; i++)
      EXPECT_EQ(screw(i) , 0);
  }

  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);


    EXPECT_EQ(screw.linear()(0) , linear(0));
    EXPECT_EQ(screw.linear()(1) , linear(1));
    EXPECT_EQ(screw.linear()(2) , linear(2));

    EXPECT_LT(fabs(screw(3) - angular.axis()(0)*angular.angle()) , 1e-16);
    EXPECT_LT(fabs(screw(4) - angular.axis()(1)*angular.angle()) , 1e-16);
    EXPECT_LT(fabs(screw(5) - angular.axis()(2)*angular.angle()) , 1e-16);
  }

  {
    Vector3D<> linear1(0.1, 0.2, 0.3);
    EAA<> angular1(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw1(linear1, angular1);

    Vector3D<> linear2(0.2, 0.3, 0.4);
    EAA<> angular2(0.5, 0.6, 0.7);
    VelocityScrew6D<> screw2(linear2, angular2);

    VelocityScrew6D<> screwA(screw1);
    VelocityScrew6D<> screwB(screw2);

    screw1 += screw2;
    for (size_t i = 0; i<6; i++) {
      EXPECT_LT(fabs(screw1(i) - (screwA(i)+screwB(i))) , 1e-15);
    }

    screw2 -= screw1;
    for (size_t i = 0; i<6; i++) {
      EXPECT_LT(fabs(screw2(i) - -screwA(i)) , 1e-15);
    }
  }

  {
    Vector3D<> linear1(1, 2, 3);
    EAA<> angular1(4, 5, 6);
    VelocityScrew6D<> screw(linear1, angular1);

	// Test unqualified lookup
    EXPECT_LT(fabs(norm1(screw)-(1+2+3+4+5+6)) , 1e-15);
    EXPECT_LT(fabs(norm2(screw)-sqrt(1.0*1+2*2+3*3+4*4+5*5+6*6)) , 1e-15);
    EXPECT_LT(fabs(normInf(screw)-6) , 1e-15);

	// Test qualified lookup
	EXPECT_LT(fabs(rw::math::norm1(screw) - (1 + 2 + 3 + 4 + 5 + 6)) , 1e-15);
	EXPECT_LT(fabs(rw::math::norm2(screw) - sqrt(1.0 * 1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5 + 6 * 6)) , 1e-15);
	EXPECT_LT(fabs(rw::math::normInf(screw) - 6) , 1e-15);
  }


  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);
    //std::cout<<"VelocityScrew<T> = "<<screw<<std::endl;
  }

  {
    Vector3D<> linear(0.1, 0.2, 0.3);
    EAA<> angular(0.4, 0.5, 0.6);
    VelocityScrew6D<> screw(linear, angular);
	{
		VelocityScrew6D<float> vsf = cast<float>(screw);
		for (size_t i = 0; i < 6; i++)
			EXPECT_EQ((float)screw(i) , vsf(i));
	}
	{
		VelocityScrew6D<float> vsf = rw::math::cast<float>(screw); // qualified lookup
		for (size_t i = 0; i < 6; i++)
			EXPECT_EQ((float)screw(i) , vsf(i));
	}
  }

  {
    const Vector3D<> linear(0.1, 0.2, 0.3);
    const EAA<> angular(0.4, 0.5, 0.6);
    const VelocityScrew6D<> screw(linear, angular);
    const Eigen::VectorXd eigen = screw.e();
    for (size_t i = 0; i<6; i++)
    	EXPECT_EQ(eigen(i) , screw(i));
  }

  {
    /* Test comparison operators operator== and operator!= */
    Vector3D<> linear1(1, 2, 3);
    EAA<> angular1(4, 5, 6);
    VelocityScrew6D<> screw1(linear1, angular1);
    Vector3D<> linear2(1, 2, 3);
    EAA<> angular2(4, 5, 6);
    VelocityScrew6D<> screw2(linear2, angular2);
    EXPECT_EQ(screw1 , screw2);
    EXPECT_TRUE(!(screw1 != screw2));
    Vector3D<> linear3(1, 4, 3);
    EAA<> angular3(4, 5, 2);
    VelocityScrew6D<> screw3(linear3, angular3);
    EXPECT_NE(screw1 , screw3);
    EXPECT_TRUE(!(screw1 == screw3));
  }

}
