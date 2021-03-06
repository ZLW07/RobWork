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

#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::geometry;
using rw::math::Vector3D;

TEST(TriangleUtilTest, DivideTriMeshBox ){
	Box box;
	TriMesh::Ptr trimesh = box.getTriMesh();
	Plane plane(Vector3D<>(0,0,1), 0.1);
	std::pair<TriMesh::Ptr, TriMesh::Ptr> frontAndBack = TriangleUtil::divide<Triangle<> >(trimesh, &plane);
	//With the current algorithm for dividing we expect there to be 14 triangles in front and behind (after they have been cut)
	EXPECT_EQ(frontAndBack.first->getSize() , 14);
	EXPECT_EQ(frontAndBack.second->getSize() , 14);

	//Check that all vertices are in front and behind, respectively
	double EPS = 1e-15;
	for (size_t i = 0; i<frontAndBack.first->getSize(); i++) {
		const Triangle<>& tri = frontAndBack.first->getTriangle(i);
		for (size_t j = 0; j<3; j++)
			EXPECT_GE(plane.distance(tri.getVertex(j)) , -EPS);
	}
	for (size_t i = 0; i<frontAndBack.second->getSize(); i++) {
		const Triangle<>& tri = frontAndBack.second->getTriangle(i);
		for (size_t j = 0; j<3; j++)
			EXPECT_LE(plane.distance(tri.getVertex(j)) , EPS);
	}
}


TEST(TriangleUtilTest, DivideTriMeshNormalTest){
	PlainTriMeshD trimesh;
	Vector3D<> v1(-1,-1,-1);
	Vector3D<> v2(1,1,1);
	Vector3D<> v3(1,0,1);
	Triangle<> tri(v1, v2, v3);
	trimesh.add(tri);
	Plane plane1(Vector3D<>(0,0,1), 0);
	Plane plane2(Vector3D<>(0,0,-1), 0);

	std::pair<TriMesh::Ptr, TriMesh::Ptr> frontAndBack1 = TriangleUtil::divide<Triangle<> >(&trimesh, &plane1);
	std::pair<TriMesh::Ptr, TriMesh::Ptr> frontAndBack2 = TriangleUtil::divide<Triangle<> >(&trimesh, &plane2);

	//With the current algorithm for dividing we expect there to be 14 triangles in front and behind (after they have been cut)
	EXPECT_EQ(frontAndBack1.first->getSize() , 2);
	EXPECT_EQ(frontAndBack1.second->getSize() , 1);

	EXPECT_EQ(frontAndBack2.first->getSize() , 1);
	EXPECT_EQ(frontAndBack2.second->getSize() , 2);
	
	
	//Check that all vertices are in front and behind, respectively
	double EPS = 1e-15;
	for (size_t i = 0; i<frontAndBack1.first->getSize(); i++) {
		const Triangle<>& tri = frontAndBack1.first->getTriangle(i);
		EXPECT_LT( (tri.calcFaceNormal() - tri.calcFaceNormal()).norm2() , EPS);
	}
	for (size_t i = 0; i<frontAndBack1.second->getSize(); i++) {
		const Triangle<>& tri = frontAndBack1.second->getTriangle(i);
		EXPECT_LT( (tri.calcFaceNormal() - tri.calcFaceNormal()).norm2() , EPS);
	}
	for (size_t i = 0; i<frontAndBack2.first->getSize(); i++) {
		const Triangle<>& tri = frontAndBack2.first->getTriangle(i);
		EXPECT_LT( (tri.calcFaceNormal() - tri.calcFaceNormal()).norm2() , EPS);
	}
	for (size_t i = 0; i<frontAndBack2.second->getSize(); i++) {
		const Triangle<>& tri = frontAndBack2.second->getTriangle(i);
		EXPECT_LT( (tri.calcFaceNormal() - tri.calcFaceNormal()).norm2() , EPS);
	}

}
