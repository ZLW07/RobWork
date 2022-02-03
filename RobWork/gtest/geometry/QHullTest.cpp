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

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/QHullND.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/VectorND.hpp>

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using namespace rw::geometry;
using rw::geometry::QHullND;
using rw::loaders::GeometryFactory;
using rw::math::VectorND;

TEST (QHullND, build2D)
{
    QHullND< 2 > qhull;
    std::vector< VectorND< 2 > > vertices (3);
    vertices[0][0] = 1;
    vertices[0][1] = 0;
    vertices[1][0] = 0;
    vertices[1][1] = 1;
    vertices[2][0] = -1;
    vertices[2][1] = -1;
    qhull.rebuild (vertices);
    const VectorND< 2 > center = qhull.getCentroid ();
    EXPECT_NEAR (
        std::sqrt (10. / 144.) - 1. / 3., center[0], std::numeric_limits< double >::epsilon ());
    EXPECT_NEAR (
        std::sqrt (10. / 144.) - 1. / 3., center[1], std::numeric_limits< double >::epsilon ());
}

/* Constructs a square in 2D and considers a set of 4 points around it.
 * Checks if the correct result is gained for:
 *  - isInside,
 *  - getMinDistInside, and
 *  - getMinDistOutside.
 */
TEST (QHullND, distanceFuncs)
{
    QHullND< 2 > qhull;
    std::vector< VectorND< 2 > > vertices (4);    // square with corners at "1,1"
    vertices[0][0] = 1;                           // top right
    vertices[0][1] = 1;
    vertices[1][0] = 1;    // bot right
    vertices[1][1] = -1;
    vertices[2][0] = -1;    // top left
    vertices[2][1] = 1;
    vertices[3][0] = -1;    // bot left
    vertices[3][1] = -1;
    qhull.rebuild (vertices);

    VectorND< 2 > inside_far_from_boarder, inside_close_to_boarder;
    inside_far_from_boarder[0] = 0.1;
    inside_far_from_boarder[1] = 0.3;
    inside_close_to_boarder[0] = -0.8;
    inside_close_to_boarder[1] = 0.55;
    VectorND< 2 > outside_far_from_boarder, outside_close_to_boarder;
    outside_far_from_boarder[0] = 3.0;
    outside_far_from_boarder[1] = 0.3;
    outside_close_to_boarder[0] = -1.3;
    outside_close_to_boarder[1] = 0.32;

    // -- test if inside/outside is right --
    EXPECT_TRUE (qhull.isInside (inside_far_from_boarder));
    EXPECT_TRUE (qhull.isInside (inside_close_to_boarder));
    EXPECT_FALSE (qhull.isInside (outside_far_from_boarder));
    EXPECT_FALSE (qhull.isInside (outside_close_to_boarder));

    // -- test outside distances --
    static const double EPSILON =
        0.000000001;    // own epsilon is used to account for precision in calculations...
    EXPECT_NEAR (qhull.getMinDistOutside (inside_far_from_boarder), 0.0, EPSILON);
    EXPECT_NEAR (qhull.getMinDistOutside (inside_close_to_boarder), 0.0, EPSILON);
    EXPECT_NEAR (qhull.getMinDistOutside (outside_far_from_boarder), 2.0, EPSILON);
    EXPECT_NEAR (qhull.getMinDistOutside (outside_close_to_boarder), 0.3, EPSILON);

    // -- test inside distances --
    EXPECT_NEAR (qhull.getMinDistInside (inside_far_from_boarder), 0.7, EPSILON);
    EXPECT_NEAR (qhull.getMinDistInside (inside_close_to_boarder), 0.2, EPSILON);


    EXPECT_NEAR(qhull.getMinDistInside(outside_far_from_boarder), 0.0, EPSILON); 
    EXPECT_NEAR(qhull.getMinDistInside(outside_close_to_boarder), 0.0, EPSILON); 
}

/* Test the volume algorithm by computing the volume of a 2D square.
 * The square has lengths 2.
 * The square is offset along x=y in positive and negative direction.
 * This is done to test if it works when the point [0,0] is NOT part of the hull.
 */
TEST (QHullND, volume_square_2D)
{
    for (int i = 0; i < 5; i++) {
        rw::geometry::QHullND< 2 > qhull;
        std::vector< rw::math::VectorND< 2 > > vertices (4);
        const double offset = (i - 2) * std::pow (1.1436, i);
        vertices[0][0]      = 1 + offset;
        vertices[0][1]      = 1 + offset;
        vertices[1][0]      = 1 + offset;
        vertices[1][1]      = -1 + offset;
        vertices[2][0]      = -1 + offset;
        vertices[2][1]      = -1 + offset;
        vertices[3][0]      = -1 + offset;
        vertices[3][1]      = 1 + offset;
        qhull.rebuild (vertices);
        const double volume = qhull.getVolume ();

        static const double EPSILON =
            0.000000001;    // own epsilon is used to account for precision in calculations...
        EXPECT_NEAR (4.0, volume, EPSILON);
    }
}

/* Tests for when the hull includes 0
 */
TEST (QHullND, volume_square_2D_zero)
{
    rw::geometry::QHullND< 2 > qhull;
    std::vector< rw::math::VectorND< 2 > > vertices (4);
    vertices[0][0] = 2;
    vertices[0][1] = 2;
    vertices[1][0] = 2;
    vertices[1][1] = 0;
    vertices[2][0] = 0;
    vertices[2][1] = 0;
    vertices[3][0] = 0;
    vertices[3][1] = 2;
    qhull.rebuild (vertices);
    const double volume = qhull.getVolume ();

    static const double EPSILON =
        0.000000001;    // own epsilon is used to account for precision in calculations...
    EXPECT_NEAR (4.0, volume, EPSILON);
}

/* Tests the volume algorithm for a n-Sphere
 * Used to test for higher dimensional QHull.
 * QHull is generated from a random set of unit vectors.
 * The volume should approximate the volume of the nSphere when enough vertices are included.
 */
TEST (QHullND, volume_nSphere)
{
    const size_t dim = 4;    // dim must be even number // if 6 or higher it will require large RAM.
    rw::geometry::QHullND< dim > qhull;
    std::vector< VectorND< dim > > vertices (
        static_cast< std::vector< VectorND< dim > >::size_type > (
            std::pow (15, dim)));    // decrease this for large dimensions...

    for (size_t v = 0; v < vertices.size (); v++) {
        vertices.at (v) = rw::math::VectorND< dim > (rw::math::Math::ranDir (dim, 1.0).e ());
    }

    qhull.rebuild (vertices);
    const double volume = qhull.getVolume ();

    // static const double EPSILON = 0.000000001; // own epsilon is used to account for precision in
    // calculations...
    const double realVolume =
        std::pow (rw::math::Pi, dim / 2) / rw::math::Math::factorial (dim / 2);
    const double range = realVolume * 0.01;
    EXPECT_NEAR (realVolume, volume, range);
}

/**
 * Test that QHull can handle the quadratic objects that is used for BV tree.
 */
TEST (QHullND, QuadraticObjectA)
{
    const Geometry::Ptr geomA = GeometryFactory::getGeometry ("#Custom QuadraticTestObjectA ");
    QHullND< 3 > qhull;
    std::vector< rw::math::VectorND< 3 > > vertecies;
	GeometryData::Ptr geoData = geomA->getGeometryData (); 

    TriMesh::Ptr mesh = geoData->getTriMesh ();

	for(size_t i = 0; i < mesh->size(); i++) {
		Triangle<double> tri;
		mesh->getTriangle(i,tri); 

		vertecies.push_back(rw::math::VectorND<3>(tri[0][0],tri[0][1],tri[0][2]));
		vertecies.push_back(rw::math::VectorND<3>(tri[1][0],tri[1][1],tri[1][2]));
		vertecies.push_back(rw::math::VectorND<3>(tri[2][0],tri[2][1],tri[2][2]));
	}
    qhull.rebuild(vertecies);

    EXPECT_DOUBLE_EQ(qhull.getVolume(),688.35963784205819);


    // CHECK INSIDE functions
    VectorND<3> c = qhull.getCentroid();
    VectorND<3> u (10,0,0);
    EXPECT_TRUE(qhull.isInside(c));

    double d = qhull.getMinDistInside(c);
    EXPECT_NEAR(d,3.4999457440957107,1e-5);
    EXPECT_EQ(qhull.getMinDistInside(u),0);
    EXPECT_TRUE(qhull.isInside(c+VectorND<3>(d-0.000001,0,0)));
    EXPECT_NEAR(qhull.getAvgDistInside(c),4.3903483324421027,1e-5);

    // CHECK OUTSIDE functions

    EXPECT_GT(qhull.getMinDistOutside(u),0);
    EXPECT_EQ(qhull.getMinDistOutside(c),0);


    // CHECK closest_point 
    std::cout << "closest_point: " << qhull.getClosestPoint(u) << std::endl;

    // CHECK storage

    EXPECT_GE(qhull.getHullVertices().size(),96);
    EXPECT_GE(qhull.getFaceIndices().size(),564);
    EXPECT_GE(qhull.getFaceNormals().size(),188);
    EXPECT_GE(qhull.getFaceOffsets().size(),188);
}