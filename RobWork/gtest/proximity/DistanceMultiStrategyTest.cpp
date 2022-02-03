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

#include <rw/core/Ptr.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/math/EAA.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <iostream>
using rw::core::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
namespace {
void checkProximityStrategyData(const ProximityStrategyData& data, const DistanceMultiStrategy::Result& res) {
	EXPECT_NE(CollisionStrategy::FirstContact,data.getCollisionQueryType());
	EXPECT_EQ(CollisionStrategy::AllContacts,data.getCollisionQueryType());
	EXPECT_EQ(res.a,data.getMultiDistanceData().a);
	EXPECT_EQ(res.b,data.getMultiDistanceData().b);
	EXPECT_EQ(res.p1,data.getMultiDistanceData().p1);
	EXPECT_EQ(res.p2,data.getMultiDistanceData().p2);
	EXPECT_EQ(res.distance,data.getMultiDistanceData().distance);
	EXPECT_EQ(res.p1s,data.getMultiDistanceData().p1s);
	EXPECT_EQ(res.p2s,data.getMultiDistanceData().p2s);
	EXPECT_EQ(res.p1prims,data.getMultiDistanceData().p1prims);
	EXPECT_EQ(res.p2prims,data.getMultiDistanceData().p2prims);
	EXPECT_EQ(res.distances,data.getMultiDistanceData().distances);
    EXPECT_EQ(0,data.rel_err);
	EXPECT_EQ(0,data.abs_err);
}

class DistanceMultiStrategyTest : public ::testing::TestWithParam<std::string> {
protected:
	DistanceMultiStrategyTest():
		frameA(new MovableFrame("FrameA")),
		frameB(new MovableFrame("FrameB")),
		tolerance(0.0005),
		eps(0.0001)
	{
	}

	void SetUp() {
		strategy = DistanceMultiStrategy::Factory::makeStrategy(GetParam());
		ASSERT_FALSE(strategy.isNull());
		sstruct.addFrame(frameA);
		sstruct.addFrame(frameB);
		modelA = strategy->createModel();
		modelB = strategy->createModel();
		data.setCollisionQueryType(CollisionStrategy::AllContacts);
	}

	void TearDown() {
		// Important: models must be destroyed as some strategies must be able to cleanup internal caches.
		// PQP models are for instance cached based on the memory address of the GeometryData object
		// (other tests might create a different geometry on the same address)
		if (!strategy.isNull()) {
			strategy->destroyModel(modelA.get());
			strategy->destroyModel(modelB.get());
		}
	}

	DistanceMultiStrategy::Ptr strategy;
	Frame* const frameA;
	Frame* const frameB;
	StateStructure sstruct;
	ProximityModel::Ptr modelA;
	ProximityModel::Ptr modelB;
	ProximityStrategyData data;
	const double tolerance;
	const double eps;
};

std::vector<std::string> strategies = DistanceMultiStrategy::Factory::getStrategies();
}

TEST(Factory, DistanceMultiStrategy) {
	EXPECT_GT(strategies.size(),0u);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		EXPECT_TRUE(DistanceMultiStrategy::Factory::hasStrategy(strategies[i]));
		EXPECT_FALSE(DistanceMultiStrategy::Factory::makeStrategy(strategies[i]).isNull());
	}
	EXPECT_FALSE(DistanceMultiStrategy::Factory::hasStrategy("dfjgkjfgæah"));
	EXPECT_TRUE(DistanceMultiStrategy::Factory::makeStrategy("dfjgkjfgæah").isNull());
}

INSTANTIATE_TEST_CASE_P(ProximityStrategy, DistanceMultiStrategyTest, ::testing::ValuesIn(strategies));

TEST_P(DistanceMultiStrategyTest, Plane_Triangle) {
	const rw::core::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const PlainTriMeshD::Ptr mesh = ownedPtr(new PlainTriMeshD());
	mesh->add(Triangle<>(Vector3D<>(0.3,-0.1,0),Vector3D<>(0.3+0.1,0,0),Vector3D<>(0.3,0.1,0)));
	//mesh->add(Triangle<>(Vector3D<>(0.3,-0.1,0),Vector3D<>(0.3+0.1,-0.1,-0.0001),Vector3D<>(0.3+0.1,0,0)));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(mesh));

	static const Transform3D<> wTa = Transform3D<>::identity();
	static const Transform3D<> wTb(Vector3D<>::z()*(tolerance-eps));
	DistanceMultiStrategy::Result res;

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	res = strategy->distances(frameA,wTa,frameB,wTb,tolerance,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	res = strategy->distances(modelA,wTa,modelB,wTb,tolerance,data);
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distance));

	// Check result
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);
	// todo: change the following from 2 to 1 when redundant pairs can be avoided in PQP
	ASSERT_EQ(2u,res.p1s.size());
	ASSERT_EQ(2u,res.p2s.size());
	ASSERT_EQ(2u,res.p1prims.size());
	ASSERT_EQ(2u,res.p2prims.size());
	ASSERT_EQ(2u,res.distances.size());
	EXPECT_EQ(res.p1,res.p1s[0]);
	EXPECT_EQ(res.p2,res.p2s[0]);
	EXPECT_EQ(res.p1[0],res.p2[0]);
	EXPECT_EQ(res.p1[1],res.p2[1]);
	// todo check if p1 and p2 is inside their triangles...
	EXPECT_FLOAT_EQ(0.f,static_cast<float>(res.p1[2]));
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.p2[2]));
	EXPECT_GE(res.p1prims[0],0); // either first triangle on plane
	EXPECT_LE(res.p1prims[0],1); // ... or second
	EXPECT_EQ(0,res.p2prims[0]); // there is only one triangle
	EXPECT_FLOAT_EQ(static_cast<float>(tolerance-eps),static_cast<float>(res.distances[0]));

	SCOPED_TRACE("Check ProximityStrategyData");
	checkProximityStrategyData(data,res);
}

TEST_P(DistanceMultiStrategyTest, Plane_Cuboid) {
	static const double s = 0.1;
	const rw::core::Ptr<Plane> plane = ownedPtr(new Plane());
	Geometry geomA(plane);
	const Box::Ptr box = ownedPtr(new Box(s,s,s));
	const Geometry::Ptr geomB = ownedPtr(new Geometry(box));

	static const Transform3D<> wTa = Transform3D<>::identity();
	static const Transform3D<> wTb(Vector3D<>::z()*(s/2+tolerance-eps));
	DistanceMultiStrategy::Result res;

	// Test the interface where models are stored internally in the strategy
	strategy->addModel(frameA,geomA);
	strategy->addModel(frameB,geomB);
	res = strategy->distances(frameA,wTa,frameB,wTb,tolerance,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());
	strategy->clearFrames();

	// Test the interface where models are stored external to the strategy
	strategy->addGeometry(modelA.get(),geomA);
	strategy->addGeometry(modelB.get(),geomB);
	res = strategy->distances(modelA,wTa,modelB,wTb,tolerance,data);
	EXPECT_NEAR(tolerance-eps,res.distance,std::numeric_limits<float>::epsilon());

	// Check result
	EXPECT_EQ(modelA,res.a);
	EXPECT_EQ(modelB,res.b);

	std::cout << "P1S: " << std::endl;
	for (size_t i = 0; i < res.p1s.size(); i++){
		std::cout << " - " << res.p1s[i] << " vs " << res.p2s[i] << " width d: " << res.distances[i] << std::endl;
	}
	// todo: change the following from 12 to 11 when redundant pairs can be avoided in PQP
	ASSERT_EQ(12u,res.p1s.size());
	ASSERT_EQ(12u,res.p2s.size());
	ASSERT_EQ(12u,res.p1prims.size());
	ASSERT_EQ(12u,res.p2prims.size());
	ASSERT_EQ(12u,res.distances.size());
	EXPECT_EQ(res.p1,res.p1s[0]);
	EXPECT_EQ(res.p2,res.p2s[0]);
	EXPECT_FLOAT_EQ(res.p1[0],res.p2[0]);
	EXPECT_FLOAT_EQ(res.p1[1],res.p2[1]);
	// todo check if p1 and p2 is inside their triangles...
	EXPECT_FLOAT_EQ(0.f,static_cast<float>(res.p1[2]));
	EXPECT_NEAR(tolerance-eps,res.p2[2],std::numeric_limits<float>::epsilon());
	//EXPECT_FLOAT_EQ(tolerance-eps,res.p2[2]);
	EXPECT_GE(res.p1prims[0],0); // either first triangle on plane
	EXPECT_LE(res.p1prims[0],1); // ... or second
	EXPECT_EQ(10,res.p2prims[0]); // there is only one triangle
	EXPECT_NEAR(tolerance-eps,res.distances[0],std::numeric_limits<float>::epsilon());
	//EXPECT_FLOAT_EQ(tolerance-eps,res.distances[0]);

	SCOPED_TRACE("Check ProximityStrategyData");
	checkProximityStrategyData(data,res);
}

TEST_P(DistanceMultiStrategyTest, Cuboids_Cuboids) {
    static const double s = 0.1;
    static const EAA<> eaa(normalize(Vector3D<>(1,1,1)),Vector3D<>::z());
    static const Rotation3D<> R(eaa.toRotation3D());
    static const Transform3D<> wTa1(Vector3D<>(-s/2, 0, 0), R);
    static const Transform3D<> wTa2(Vector3D<>(+s/2, 0, 0), R);
    static const Transform3D<> wTb1(Vector3D<>(+s/2, 0, 0), R);
    static const Transform3D<> wTb2(Vector3D<>(-s/2, 0, eps), R);

    const Box::Ptr box = ownedPtr(new Box(s,s,s));
    const Geometry::Ptr geomA1 = ownedPtr(new Geometry(box, wTa1));
    const Geometry::Ptr geomA2 = ownedPtr(new Geometry(box, wTa2));
    const Geometry::Ptr geomB1 = ownedPtr(new Geometry(box, wTb1));
    const Geometry::Ptr geomB2 = ownedPtr(new Geometry(box, wTb2));

    static const Transform3D<> wTa(Vector3D<>(0, 0, +(std::sqrt(3)*s+tolerance-eps)/2));
    static const Transform3D<> wTb(Vector3D<>(0, 0, -(std::sqrt(3)*s+tolerance-eps)/2));
    DistanceMultiStrategy::Result res;

    // Test the interface where models are stored external to the strategy
    strategy->addGeometry(modelA.get(), geomA1);
    strategy->addGeometry(modelA.get(), geomA2);
    strategy->addGeometry(modelB.get(), geomB1);
    strategy->addGeometry(modelB.get(), geomB2);

    res = strategy->distances(modelA, wTa, modelB, wTb, tolerance, data);

    // Check result
    EXPECT_EQ(modelA, res.a);
    EXPECT_EQ(modelB, res.b);
    // Points: 5 points to 6 for each pair of cuboid: 2*(5+6)=22 point pairs.
    // todo: change the following from 22 to 21 when redundant pairs can be avoided in PQP
    ASSERT_EQ(22u,res.p1s.size());
    ASSERT_EQ(22u,res.p2s.size());
    ASSERT_EQ(22u,res.geoIdxA.size());
    ASSERT_EQ(22u,res.geoIdxB.size());
    ASSERT_EQ(22u,res.p1prims.size());
    ASSERT_EQ(22u,res.p2prims.size());
    ASSERT_EQ(22u,res.distances.size());
    EXPECT_EQ(res.p1,res.p1s[0]);
    EXPECT_EQ(res.p2,res.p2s[0]);
    EXPECT_NEAR(-s/2, res.p1[0],std::numeric_limits<float>::epsilon());
    EXPECT_NEAR(-s/2, res.p2[0],std::numeric_limits<float>::epsilon());
    EXPECT_NEAR(+(tolerance-eps)/2, res.p1[2],std::numeric_limits<float>::epsilon());
    EXPECT_NEAR(-(tolerance-eps)/2+eps, res.p2[2],std::numeric_limits<float>::epsilon());
    for (std::size_t i = 0; i < res.p1s.size(); i++) {
        std::stringstream sstr;
        sstr << "Pair number " << i;
        SCOPED_TRACE(sstr.str());
        EXPECT_GE(res.geoIdxA[i],0);
        EXPECT_LE(res.geoIdxA[i],1);
        EXPECT_GE(res.geoIdxB[i],0);
        EXPECT_LE(res.geoIdxB[i],1);
        EXPECT_GE(res.p1prims[i],0);
        EXPECT_LE(res.p1prims[i],11);
        EXPECT_GE(res.p2prims[i],0);
        EXPECT_LE(res.p2prims[i],11);
        EXPECT_NEAR(+(tolerance-eps)/2, res.p1s[i][2],std::numeric_limits<float>::epsilon());
        if (res.geoIdxA[i] == 0) {
            EXPECT_EQ(res.geoIdxB[i], 1);
            EXPECT_NEAR(tolerance-eps-eps, res.distances[i], std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(-s/2, res.p1s[i][0],std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(-s/2, res.p2s[i][0],std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(-(tolerance-eps)/2+eps, res.p2s[i][2],std::numeric_limits<float>::epsilon());
        } else if (res.geoIdxA[i] == 1) {
            EXPECT_EQ(res.geoIdxB[i], 0);
            EXPECT_NEAR(tolerance-eps, res.distances[i], std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(s/2, res.p1s[i][0],std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(s/2, res.p2s[i][0],std::numeric_limits<float>::epsilon());
            EXPECT_NEAR(-(tolerance-eps)/2, res.p2s[i][2],std::numeric_limits<float>::epsilon());
        }
    }

    SCOPED_TRACE("Check ProximityStrategyData");
    checkProximityStrategyData(data,res);
}
