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

#include "../TestEnvironment.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/ProximityCalculator.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <gtest/gtest.h>

using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::proximity;
using namespace std;
using namespace rwlibs::proximitystrategies;

TEST (ProximityCalculator, DistanceStrategyConstructor)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/DistanceTest.wc.xml");

    ProximityCalculator< DistanceStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultDistanceStrategy ());

    EXPECT_DOUBLE_EQ (calc.getComputationTime (), 0.0);
    EXPECT_EQ (calc.getNoOfCalls (), 0);

    std::string IDs[4] = {"Box1d", "Box2d", "Box3d", "Box4d"};
    int i              = 0;
    for (Frame* f : wc->findFrames< Frame > ()) {
        if (f->getName () == "WORLD") {
            continue;
        }
        std::vector< std::string > geoID = calc.getGeometryIDs (f);
        EXPECT_EQ (geoID.size (), 1u);
        EXPECT_EQ (IDs[i++], geoID[0]);
    }
}

TEST (ProximityCalculator, DistanceMultiStrategyConstructor)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/DistanceTest.wc.xml");

    ProximityCalculator< DistanceMultiStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultDistanceMultiStrategy ());

    EXPECT_DOUBLE_EQ (calc.getComputationTime (), 0.0);
    EXPECT_EQ (calc.getNoOfCalls (), 0);

    std::string IDs[4] = {"Box1d", "Box2d", "Box3d", "Box4d"};
    int i              = 0;
    for (Frame* f : wc->findFrames< Frame > ()) {
        if (f->getName () == "WORLD") {
            continue;
        }
        std::vector< std::string > geoID = calc.getGeometryIDs (f);
        EXPECT_EQ (geoID.size (), 1u);
        EXPECT_EQ (IDs[i++], geoID[0]);
    }
}

TEST (ProximityCalculator, CollisionStrategyConstructor)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/CollisionTest.wc.xml");

    ProximityCalculator< CollisionStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    EXPECT_DOUBLE_EQ (calc.getComputationTime (), 0.0);
    EXPECT_EQ (calc.getNoOfCalls (), 0);

    std::string IDs[4] = {"Sphere1", "Box1", "Sphere2", "Box2"};
    int i              = 0;
    for (Frame* f : wc->findFrames< Frame > ()) {
        if (f->getName () == "WORLD") {
            continue;
        }
        std::vector< std::string > geoID = calc.getGeometryIDs (f);
        EXPECT_EQ (geoID.size (), 1u);
        EXPECT_EQ (geoID[0], f->getName ());
        EXPECT_EQ (IDs[i++], geoID[0]);
    }
}

TEST (ProximityCalculator, CollisionCalculate)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/CollisionTest.wc.xml");

    ProximityCalculator< CollisionStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    State state                         = wc->getDefaultState ();
    ProximityStrategyData::Ptr settings = ownedPtr (new ProximityStrategyData ());
    settings->setCollisionQueryType (rw::proximity::CollisionStrategy::QueryType::AllContacts);
    ProximityStrategyData::PtrList result = ownedPtr (new ProximityStrategyData::List ());

    ProximityStrategyData res = calc.calculate (state, settings, result);

    EXPECT_TRUE (res.inCollision ());
    EXPECT_EQ (result->size (), 1u);
    EXPECT_TRUE (result->at (0).inCollision ());
}

TEST (ProximityCalculator, DistanceCalculate)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/DistanceTest.wc.xml");
    ProximityCalculator< DistanceStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultDistanceStrategy ());

    State state                           = wc->getDefaultState ();
    ProximityStrategyData::Ptr settings   = NULL;
    ProximityStrategyData::PtrList result = ownedPtr (new ProximityStrategyData::List ());

    ProximityStrategyData res = calc.calculate (state, settings, result);

    EXPECT_EQ (result->size (), 6u);

    // Distance from Sphere1 - Box1
    //  distance = center_distance{1} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR (result->at (0).getDistanceData ().distance, 0.7, 0.0001);

    // Distance from sphere1 to sphere2:
    //  distance = center_dist{sqrt(1²+2²+2²)}- 2* radius{0.2}
    EXPECT_NEAR (result->at (1).getDistanceData ().distance, 2.6, 0.001);

    // Distance from sphere1 to box2:
    //  distance = center_distance{2} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR (result->at (2).getDistanceData ().distance, 1.7, 0.001);

    // Distance from box1 to box2:
    //  distance = center_distance{2-1}-2*side_length{0.2}/2
    EXPECT_NEAR (result->at (4).getDistanceData ().distance, 0.8, 0.001);
}

TEST (ProximityCalculator, MultiDistanceCalculate)
{
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                      "/workcells/DistanceTest.wc.xml");

    ProximityCalculator< DistanceMultiStrategy > calc (
        wc, ProximityStrategyFactory::makeDefaultDistanceMultiStrategy ());

    State state                           = wc->getDefaultState ();
    ProximityStrategyData::Ptr settings   = ownedPtr (new ProximityStrategyData ());
    ProximityStrategyData::PtrList result = ownedPtr (new ProximityStrategyData::List ());
    settings->setMultiDistanceTolerance (2.61);

    ProximityStrategyData res = calc.calculate (state, settings, result);

    EXPECT_EQ (result->size (), 6u);

    DistanceMultiStrategy::Result res0 = result->at (0).getMultiDistanceData ();
    // Distance from Sphere1 - Box1
    //  distance = center_distance{1} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR ((res0.p1 - res0.p2).norm2 (), 0.7, 0.0001);

    DistanceMultiStrategy::Result res1 = result->at (1).getMultiDistanceData ();
    // Distance from sphere1 to sphere2:
    //  distance = center_dist{sqrt(1²+2²+2²)}- 2* radius{0.2}
    EXPECT_NEAR ((res1.p1 - res1.p2).norm2 (), 2.6, 0.001);

    DistanceMultiStrategy::Result res2 = result->at (2).getMultiDistanceData ();
    // Distance from sphere1 to box2:
    //  distance = center_distance{2} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR ((res2.p1 - res2.p2).norm2 (), 1.7, 0.001);

    DistanceMultiStrategy::Result res4 = result->at (4).getMultiDistanceData ();
    // Distance from box1 to box2:
    //  distance = center_distance{2-1}-2*side_length{0.2}/2
    EXPECT_NEAR ((res4.p1 - res4.p2).norm2 (), 0.8, 0.001);
}
