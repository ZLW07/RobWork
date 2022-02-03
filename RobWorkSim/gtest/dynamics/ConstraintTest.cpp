/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/Constraint.hpp>

using namespace rw::common;
using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsim::dynamics;

TEST(ConstraintTest, Constraint) {

	const MovableFrame::Ptr frameA = ownedPtr(new MovableFrame("BodyA"));
	const MovableFrame::Ptr frameB = ownedPtr(new MovableFrame("BodyB"));
	const RigidObject::Ptr objectA = ownedPtr(new RigidObject(frameA.get()));
	const RigidObject::Ptr objectB = ownedPtr(new RigidObject(frameB.get()));
	RigidBody bodyA(BodyInfo(),objectA);
	RigidBody bodyB(BodyInfo(),objectB);

	Constraint::Limit limit;
	limit.lowOn = true;
	limit.low = -1.0;

	const Transform3D<> pTc(Vector3D<>(-1.1,2.2,-3.3),EAA<>(0,-Pi,0).toRotation3D());

	{
		Constraint c("Constraint",Constraint::Fixed,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Fixed);
		EXPECT_EQ(c.getDOF(), 0);
		EXPECT_EQ(c.getDOFLinear(), 0);
		EXPECT_EQ(c.getDOFAngular(), 0);
	}
	{
		Constraint c("Constraint",Constraint::Prismatic,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Prismatic);
		EXPECT_EQ(c.getDOF(), 1);
		EXPECT_EQ(c.getDOFLinear(), 1);
		EXPECT_EQ(c.getDOFAngular(), 0);

		// Check limits
		c.setLimit(0,limit);
		EXPECT_EQ(c.getLimit(0).high , limit.high);
		EXPECT_EQ(c.getLimit(0).low , limit.low);
		EXPECT_EQ(c.getLimit(0).highOn , limit.highOn);
		EXPECT_EQ(c.getLimit(0).lowOn , limit.lowOn);
	}
	{
		Constraint c("Constraint",Constraint::Revolute,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Revolute);
		EXPECT_EQ(c.getDOF(), 1);
		EXPECT_EQ(c.getDOFLinear(), 0);
		EXPECT_EQ(c.getDOFAngular(), 1);

		// Check transform
		c.setTransform(pTc);
		EXPECT_EQ(c.getTransform() , pTc);
	}
	{
		Constraint c("Constraint",Constraint::Universal,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Universal);
		EXPECT_EQ(c.getDOF(), 2);
		EXPECT_EQ(c.getDOFLinear(), 0);
		EXPECT_EQ(c.getDOFAngular(), 2);

		// Check body pointers
		EXPECT_EQ(c.getBody1() , &bodyA);
		EXPECT_EQ(c.getBody2() , &bodyB);
	}
	{
		Constraint c("Constraint",Constraint::Spherical,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Spherical);
		EXPECT_EQ(c.getDOF(), 3);
		EXPECT_EQ(c.getDOFLinear(), 0);
		EXPECT_EQ(c.getDOFAngular(), 3);
	}
	{
		Constraint c("Constraint",Constraint::Piston,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Piston);
		EXPECT_EQ(c.getDOF(), 2);
		EXPECT_EQ(c.getDOFLinear(), 1);
		EXPECT_EQ(c.getDOFAngular(), 1);
	}
	{
		Constraint c("Constraint",Constraint::PrismaticRotoid,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::PrismaticRotoid);
		EXPECT_EQ(c.getDOF(), 2);
		EXPECT_EQ(c.getDOFLinear(), 1);
		EXPECT_EQ(c.getDOFAngular(), 1);

		// Check spring
		Constraint::SpringParams spring;
		spring.enabled = true;
		spring.compliance = Eigen::MatrixXd::Constant(2,2,1.234);
		spring.damping = Eigen::MatrixXd::Constant(2,2,4.321);
		c.setSpringParams(spring);
		EXPECT_TRUE(c.getSpringParams().enabled);
		EXPECT_NEAR(c.getSpringParams().compliance(1,0), 1.234, std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(c.getSpringParams().damping(0,1), 4.321, std::numeric_limits<double>::epsilon());
	}
	{
		Constraint c("Constraint",Constraint::PrismaticUniversal,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::PrismaticUniversal);
		EXPECT_EQ(c.getDOF(), 3);
		EXPECT_EQ(c.getDOFLinear(), 1);
		EXPECT_EQ(c.getDOFAngular(), 2);

		// Check no spring
		EXPECT_TRUE(!c.getSpringParams().enabled);
		// Check no limits
		EXPECT_FALSE(c.getLimit(0).highOn);
		EXPECT_FALSE(c.getLimit(0).lowOn);
		EXPECT_FALSE(c.getLimit(1).highOn);
		EXPECT_FALSE(c.getLimit(1).lowOn);
		EXPECT_FALSE(c.getLimit(2).highOn);
		EXPECT_FALSE(c.getLimit(2).lowOn);
	}
	{
		Constraint c("Constraint",Constraint::Free,&bodyA,&bodyB);
		EXPECT_EQ(c.getType() , Constraint::Free);
		EXPECT_EQ(c.getDOF(), 6);
		EXPECT_EQ(c.getDOFLinear(), 3);
		EXPECT_EQ(c.getDOFAngular(), 3);

		// Check limits
		c.setLimit(5,limit);
		EXPECT_EQ(c.getLimit(5).high, limit.high);
		EXPECT_EQ(c.getLimit(5).low , limit.low);
		EXPECT_EQ(c.getLimit(5).highOn , limit.highOn);
		EXPECT_EQ(c.getLimit(5).lowOn , limit.lowOn);
	}
}
