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

#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/core/Ptr.hpp>

using rw::core::ownedPtr;
using rw::geometry::Triangle;
using rw::geometry::IndexedTriangle;
using rw::geometry::IndexedTriMeshN0;
using rw::math::Vector3D;

template <typename A, typename B>
struct TemplateParameters {
  typedef A First;
  typedef B Second;
};

template <typename T>
class IndexedTriMeshTest: public ::testing::Test {
public:
	IndexedTriMeshTest() {}
	virtual ~IndexedTriMeshTest() {}
};

typedef ::testing::Types<
		TemplateParameters<float,uint64_t>, TemplateParameters<float,uint32_t>, TemplateParameters<float,uint16_t>, TemplateParameters<float,uint8_t>,
		TemplateParameters<double,uint64_t>, TemplateParameters<double,uint32_t>, TemplateParameters<double,uint16_t>, TemplateParameters<double,uint8_t> > MyTypes;
TYPED_TEST_CASE(IndexedTriMeshTest, MyTypes);

TYPED_TEST(IndexedTriMeshTest, GetTriangleTest) {
	// Test of out of bounds memory problem in IndexedTriMesh::getTriangle
	// Notice that vertex and triangle arrays are allocated with exact size, to be able to detect error using valgrind.
	typedef typename TypeParam::First T;
	typedef typename TypeParam::Second S;
	typedef IndexedTriMeshN0<T,S> Mesh;
	typedef typename Mesh::VertexArray VertexArray;
	typedef typename Mesh::TriangleArray TriangleArray;
	const Vector3D<T> vertex0(static_cast<T>(0.0),static_cast<T>(0.1),static_cast<T>(0.2));
	const Vector3D<T> vertex1(static_cast<T>(0.3),static_cast<T>(0.4),static_cast<T>(0.5));
	const Vector3D<T> vertex2(static_cast<T>(0.6),static_cast<T>(0.7),static_cast<T>(0.8));
	const Vector3D<T> vertex3(static_cast<T>(0.9),static_cast<T>(1.0),static_cast<T>(1.1));
	const rw::core::Ptr<VertexArray> vertices = ownedPtr(new VertexArray(4)); // allocation of exact size is important
	(*vertices)[0] = vertex0;
	(*vertices)[1] = vertex1;
	(*vertices)[2] = vertex2;
	(*vertices)[3] = vertex3;
	const rw::core::Ptr<TriangleArray> triangles = ownedPtr(new TriangleArray(3)); // allocation of exact size is important
	(*triangles)[0] = IndexedTriangle<S>(0,1,2);
	(*triangles)[1] = IndexedTriangle<S>(2,3,1);
	(*triangles)[2] = IndexedTriangle<S>(3,2,1);
	//triangles->reserve(4);
	const Mesh mesh(vertices,triangles);

	// Check that the triangles can be retrieved
	Triangle<T> tri;
	mesh.getTriangle(0,tri);
	EXPECT_EQ((tri[0]-vertex0).normInf(),0);
	EXPECT_EQ((tri[1]-vertex1).normInf(),0);
	EXPECT_EQ((tri[2]-vertex2).normInf(),0);
	mesh.getTriangle(1,tri);
	EXPECT_EQ((tri[0]-vertex2).normInf(),0);
	EXPECT_EQ((tri[1]-vertex3).normInf(),0);
	EXPECT_EQ((tri[2]-vertex1).normInf(),0);
	mesh.getTriangle(2,tri);
	EXPECT_EQ((tri[0]-vertex3).normInf(),0);
	EXPECT_EQ((tri[1]-vertex2).normInf(),0);
	EXPECT_EQ((tri[2]-vertex1).normInf(),0);
}

TYPED_TEST(IndexedTriMeshTest, GetIndexedTriangleTest) {
	typedef typename TypeParam::First T;
	typedef typename TypeParam::Second S;
	typedef IndexedTriMeshN0<T,S> Mesh;
	typedef typename Mesh::VertexArray VertexArray;
	typedef typename Mesh::TriangleArray TriangleArray;
	const Vector3D<T> vertex0(0.0,0.1,0.2);
	const Vector3D<T> vertex1(0.3,0.4,0.5);
	const Vector3D<T> vertex2(0.6,0.7,0.8);
	const Vector3D<T> vertex3(0.9,1.0,1.1);
	const rw::core::Ptr<VertexArray> vertices = ownedPtr(new VertexArray(4)); // allocation of exact size is important
	(*vertices)[0] = vertex0;
	(*vertices)[1] = vertex1;
	(*vertices)[2] = vertex2;
	(*vertices)[3] = vertex3;
	const rw::core::Ptr<TriangleArray> triangles = ownedPtr(new TriangleArray(3)); // allocation of exact size is important
	(*triangles)[0] = IndexedTriangle<S>(0,1,2);
	(*triangles)[1] = IndexedTriangle<S>(2,3,1);
	(*triangles)[2] = IndexedTriangle<S>(3,2,1);
	//triangles->reserve(4);
	const Mesh mesh(vertices,triangles);

	// Check that the triangles can be retrieved
	IndexedTriangle<uint32_t> tri;
	tri = mesh.getIndexedTriangle(0);
	EXPECT_EQ(tri[0],0u);
	EXPECT_EQ(tri[1],1u);
	EXPECT_EQ(tri[2],2u);
	tri = mesh.getIndexedTriangle(1);
	EXPECT_EQ(tri[0],2u);
	EXPECT_EQ(tri[1],3u);
	EXPECT_EQ(tri[2],1u);
	tri = mesh.getIndexedTriangle(2);
	EXPECT_EQ(tri[0],3u);
	EXPECT_EQ(tri[1],2u);
	EXPECT_EQ(tri[2],1u);
}
