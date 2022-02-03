/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw::math;
using namespace rw::trajectory;

Vector3D<> toVector3D(const EAA<>& eaa) {
	return Vector3D<>(eaa[0],eaa[1],eaa[2]);
}

Vector3D<> eaaVec(const Rotation3D<>& T) {
	return toVector3D(EAA<>(T));
}

Vector3D<> eaaVec(const Rotation3D<>& T1, const Rotation3D<>& T2) {
	return toVector3D(EAA<>(inverse(T1)*T2));
}

Transform3D<> halfTransform(const Transform3D<>& T1, const Transform3D<>& T2) {
	return Transform3D<>((T2.P()+T1.P())/2.,T1.R()*EAA<>(eaaVec(T1.R(),T2.R())/2.).toRotation3D());
}

TEST(TrajectoryFactoryTest, TrajFactoryTest ){

	{
		Transform3DPath path;
		std::vector<double> times;
		const Transform3D<> T1 = Transform3D<>::identity();
		const Transform3D<> T2(Vector3D<>(0.2,0.3,0.4), EAA<>(0,0,45*Deg2Rad).toRotation3D());
		const Transform3D<> T3(Vector3D<>(-0.2,0.3,0.7), EAA<>(45*Deg2Rad,0,45*Deg2Rad).toRotation3D());
		const Transform3D<> T4 = T1;
		const double t1 = 0.3;
		const double t2 = 2.0;
		const double t3 = 2.5;
		path.push_back(T1);
		path.push_back(T2);
		path.push_back(T3);
		path.push_back(T4);
		times.push_back(t1);
		times.push_back(t2);
		times.push_back(t3);
		const Transform3DTrajectory::Ptr traj = TrajectoryFactory::makeLinearTrajectory(path, times);
		// Check time
		EXPECT_NEAR(traj->startTime(),0.0,std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(traj->duration(),t1+t2+t3,std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(traj->endTime(),t1+t2+t3,std::numeric_limits<double>::epsilon());
		// Check positions
		EXPECT_TRUE(traj->x(0).equal(T1));
		EXPECT_TRUE(traj->x(t1).equal(T2));
		EXPECT_TRUE(traj->x(t1+t2).equal(T3));
		EXPECT_TRUE(traj->x(t1+t2+t3).equal(T4));
		EXPECT_TRUE(traj->x(t1/2.).equal(halfTransform(T1,T2)));
		std::cout << traj->x(t1+t2/2.) << " " << halfTransform(T2,T3) << std::endl;
		std::cout << traj->x(t1+t2+t3/2.) << " " << halfTransform(T3,T4) << std::endl;
		EXPECT_TRUE(traj->x(t1+t2/2.).equal(halfTransform(T2,T3)));
		EXPECT_TRUE(traj->x(t1+t2+t3/2.).equal(halfTransform(T3,T4)));
		// Check velocity
		const Vector3D<> linVel1 = (T2.P()-T1.P())/t1;
		const Vector3D<> linVel2 = (T3.P()-T2.P())/t2;
		const Vector3D<> linVel3 = (T4.P()-T3.P())/t3;
		EXPECT_LT((traj->dx(t1/2.).P()-linVel1).normInf() , linVel1.normInf()*std::numeric_limits<double>::epsilon());
		EXPECT_LT((traj->dx(t1+t2/2.).P()-linVel2).normInf() , linVel2.normInf()*std::numeric_limits<double>::epsilon());
		EXPECT_LT((traj->dx(t1+t2+t3/2.).P()-linVel3).normInf() , linVel3.normInf()*std::numeric_limits<double>::epsilon());
		const Vector3D<> angVel1 = eaaVec(T1.R(),T2.R())/t1;
		const Vector3D<> angVel2 = eaaVec(T2.R(),T3.R())/t2;
		const Vector3D<> angVel3 = eaaVec(T3.R(),T4.R())/t3;
		EXPECT_LT(angVel1.norm2() , Pi); // - note that angular part of dx is wrong when velocity is bigger than Pi
		EXPECT_LT(angVel2.norm2() , Pi);
		EXPECT_LT(angVel3.norm2() , Pi);
		EXPECT_LT((eaaVec((traj->dx(t1/2.).R()))-angVel1).normInf() , 1e-15);
		EXPECT_LT((eaaVec((traj->dx(t1+t2/2.).R()))-angVel2).normInf() , 1e-15);
		EXPECT_LT((eaaVec((traj->dx(t1+t2+t3/2.).R()))-angVel3).normInf() , 1e-15);
		// Check that there are no acceleration
		EXPECT_EQ(traj->ddx(t1/2.).P().normInf(),0);
		EXPECT_EQ(traj->ddx(t1+t2/2.).P().normInf(),0);
		EXPECT_EQ(traj->ddx(t1+t2+t3/2.).P().normInf(),0);
		EXPECT_TRUE(traj->ddx(t1/2.).R().equal(Rotation3D<>::identity()));
		EXPECT_TRUE(traj->ddx(t1+t2/2.).R().equal(Rotation3D<>::identity()));
		EXPECT_TRUE(traj->ddx(t1+t2+t3/2.).R().equal(Rotation3D<>::identity()));
	}
}
