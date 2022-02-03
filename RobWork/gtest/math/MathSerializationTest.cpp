/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/common/INIArchive.hpp>
#include <rw/common/BINArchive.hpp>

#include <rw/math.hpp>

#include <sstream>

using namespace rw::common;
using namespace rw::math;

const static double EPS = 5*std::numeric_limits<double>::epsilon();
const static double EPS_ROT = 1e-5;

TEST(MathSerializeTest, MathSerializationTest) {

    std::list<Archive*> archives;
    archives.push_back(new INIArchive());
    archives.push_back(new BINArchive());

    for(Archive* const archive : archives) {
        InputArchive* const iar = dynamic_cast<InputArchive*>(archive);
        OutputArchive* const oar = dynamic_cast<OutputArchive*>(archive);
        if (iar == NULL || oar == NULL)
        	continue;

    	{
    		const CameraMatrix<> write(1.,2.,3.,4.,5.,6.,7.,8.,9.);
    		CameraMatrix<> read(0,0,0,0,0,0,0,0,0);
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write(0,0)-read(0,0), EPS);
    		EXPECT_LT(write(0,1)-read(0,1), EPS);
    		EXPECT_LT(write(0,2)-read(0,2), EPS);
    		EXPECT_LT(write(1,0)-read(1,0), EPS);
    		EXPECT_LT(write(1,1)-read(1,1), EPS);
    		EXPECT_LT(write(1,2)-read(1,2), EPS);
    		EXPECT_LT(write(2,0)-read(2,0), EPS);
    		EXPECT_LT(write(2,1)-read(2,1), EPS);
    		EXPECT_LT(write(2,2)-read(2,2), EPS);
    	}

    	{
    		const EAA<> write(1.2,2.3,3.4);
    		EAA<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write[0]-read[0], EPS);
    		EXPECT_LT(write[1]-read[1], EPS);
    		EXPECT_LT(write[2]-read[2], EPS);
    	}

    	{
    		const Rotation3D<> R = EAA<>(0.1,0.2,0.3).toRotation3D();
    		const InertiaMatrix<> write(R.e());
    		InertiaMatrix<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(Rotation3D<>(write.e()).equal(Rotation3D<>(read.e()),EPS_ROT));
    	}
		
    	{
    		const Rotation3D<> R = EAA<>(0.1,0.2,0.3).toRotation3D();
    		const Jacobian write(R.e());
    		Jacobian read(3,3);
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(Rotation3D<>(write.e()).equal(Rotation3D<>(read.e()),EPS_ROT));
    	}

    	{
    		const Line2D write(1.2,2.3,3.4,4.5);
    		Line2D read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write.p1()-read.p1()).normInf(), EPS);
    		EXPECT_LT((write.p2()-read.p2()).normInf(), EPS);
    	}

    	{
    		const Line2DPolar write(1.2,2.3);
    		Line2DPolar read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write.getRho()-read.getRho(), EPS);
    		EXPECT_LT(write.getTheta()-read.getTheta(), EPS);
    	}

    	{
    		const PerspectiveTransform2D<> write(EAA<>(0.1,0.2,0.3).toRotation3D().e());
    		PerspectiveTransform2D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(Rotation3D<>(write.e()).equal(Rotation3D<>(read.e()), EPS_ROT));
    	}

    	{
    		Polynomial<> write(4);
    		write[0] = 1.2;
    		write[1] = 2.3;
    		write[2] = 3.4;
    		write[3] = 4.5;
    		write[4] = 5.6;
    		Polynomial<> read(4);
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write[0]-read[0], EPS);
    		EXPECT_LT(write[1]-read[1], EPS);
    		EXPECT_LT(write[2]-read[2], EPS);
    		EXPECT_LT(write[3]-read[3], EPS);
    		EXPECT_LT(write[4]-read[4], EPS);
    	}

    	{
    		const Pose2D<> write(1.2,2.3,3.4);
    		Pose2D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write[0]-read[0], EPS);
    		EXPECT_LT(write[1]-read[1], EPS);
    		EXPECT_LT(write[2]-read[2], EPS);
    	}

    	{
    		const Pose6D<> write(Vector3D<>(1.2,2.3,3.4),EAA<>(0.1,0.2,0.3));
    		Pose6D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(write.toTransform3D().equal(read.toTransform3D(), EPS));
    	}

    	{
    		const Q write(4,1.2,2.3,3.4,4.5);
    		Q read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}

    	{
    		const Quaternion<> write(EAA<>(0.1,0.2,0.3).toRotation3D());
    		Quaternion<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write(0)-read(0), EPS_ROT);
    		EXPECT_LT(write(1)-read(1), EPS_ROT);
    		EXPECT_LT(write(2)-read(2), EPS_ROT);
    		EXPECT_LT(write(3)-read(3), EPS_ROT);
    	}

    	{
    		const Rotation2D<> write(0.1);
    		Rotation2D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write(0,0)-read(0,0), EPS_ROT);
    		EXPECT_LT(write(0,1)-read(0,1), EPS_ROT);
    		EXPECT_LT(write(1,0)-read(1,0), EPS_ROT);
    		EXPECT_LT(write(1,1)-read(1,1), EPS_ROT);
    	}

    	{
    		const Rotation3D<> write = EAA<>(0.1,0.2,0.3).toRotation3D();
    		Rotation3D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(write.equal(read, EPS_ROT));
    	}

    	{
    		const RPY<> write(1.2,2.3,3.4);
    		RPY<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT(write[0]-read[0], EPS_ROT);
    		EXPECT_LT(write[1]-read[1], EPS_ROT);
    		EXPECT_LT(write[2]-read[2], EPS_ROT);
    	}

    	{
    		const Transform2D<> write(Vector2D<>(1.2,2.3),Rotation2D<>(0.1));
    		Transform2D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write.P()-read.P()).normInf(), EPS);
    		EXPECT_LT(write.R()(0,0)-read.R()(0,0), EPS_ROT);
    		EXPECT_LT(write.R()(0,1)-read.R()(0,1), EPS_ROT);
    		EXPECT_LT(write.R()(1,0)-read.R()(1,0), EPS_ROT);
    		EXPECT_LT(write.R()(1,1)-read.R()(1,1), EPS_ROT);
    	}

    	{
    		const Transform3D<> write(Vector3D<>(1.2,2.3,3.4),EAA<>(0.1,0.2,0.3).toRotation3D());
    		Transform3D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_TRUE(write.equal(read, EPS_ROT));
    	}

    	{
    		const Vector2D<> write(1.2,2.3);
    		Vector2D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}

    	{
    		const Vector3D<> write(1.2,2.3,3.4);
    		Vector3D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}

    	{
    		VectorND<5> write;
    		write[0] = 1.2;
    		write[1] = 2.3;
    		write[2] = 3.4;
    		write[3] = 4.5;
    		write[4] = 5.6;
    		VectorND<5> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}

    	{
    		const VelocityScrew6D<> write(1.2,2.3,3.4,4.5,5.6,6.7);
    		VelocityScrew6D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}

    	{
    		const Wrench6D<> write(1.2,2.3,3.4,4.5,5.6,6.7);
    		Wrench6D<> read;
    		std::stringstream stream;
    		archive->open(stream);
    		oar->write(write,"ID");
    		iar->read(read,"ID");
    		EXPECT_LT((write-read).normInf(), EPS);
    	}
    }

    for(Archive* const archive : archives) {
    	delete archive;
    }
}
