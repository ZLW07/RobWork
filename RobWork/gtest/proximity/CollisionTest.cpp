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

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

#include <boost/bind.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rwlibs::proximitystrategies;


std::vector< PlannerConstraint >
getConstraints (const std::vector< CollisionStrategy::Ptr >& strategies, WorkCell::Ptr workcell,
                Device::Ptr device, const State& state)
{
    std::vector< PlannerConstraint > result;
    for (const CollisionStrategy::Ptr& strategy : strategies) {
        strategy->clear ();    // Sigh.
        result.push_back (PlannerConstraint::make (strategy, workcell, device, state));
    }
    return result;
}

void testStrategy0 (const CollisionStrategy::Ptr& strategy)
{

    strategy->clear ();

    const Transform3D<> id = Transform3D<>::identity ();

    FixedFrame* o1 = new FixedFrame ("Object1", id);
    FixedFrame* o2 = new FixedFrame ("Object2", id);

    StateStructure tree;
    Frame* world = tree.getRoot ();
    tree.addFrame (o1, world);
    tree.addFrame (o2, world);

    Geometry::Ptr geom = GeometryFactory::load ("#Cylinder 0.12 0.2 8");
    strategy->addModel (o1, geom);

    strategy->addModel (o2, geom);

    const Transform3D<> a (Vector3D<> (0.1, 0.0, 0.0));
    EXPECT_TRUE (strategy->inCollision (o1, a, o2, id));

    const Transform3D<> b (Vector3D<> (10.0, 0.0, 0.0));
    EXPECT_TRUE (!strategy->inCollision (o1, b, o2, id));
}

void testStrategy1 (const CollisionStrategy::Ptr& strategy, int i)
{

    strategy->clear ();

    MovableFrame* cube1 = new MovableFrame ("cube1");
    MovableFrame* cube2 = new MovableFrame ("cube2");

    StateStructure tree;
    Frame* world = tree.getRoot ();
    tree.addFrame (cube1, world);
    tree.addFrame (cube2, world);
    WorkCell workcell (&tree, "testStrategy");

    const Transform3D<> id = Transform3D<>::identity ();
    State state            = workcell.getDefaultState ();
    cube1->setTransform (id, state);
    cube2->setTransform (id, state);

    Geometry::Ptr geom = GeometryFactory::load ("#Box 0.2 0.2 0.2");
    bool result;

    CollisionDetector detector (&workcell, strategy);
    detector.addGeometry (cube1, geom);
    detector.addGeometry (cube2, geom);
    detector.addRule (ProximitySetupRule::makeInclude (cube1->getName (), cube2->getName ()));

    result = detector.inCollision (state);
    EXPECT_TRUE(result);

    cube1->setTransform (Transform3D<> (Vector3D<> (10.0, 0.0, 0.0)), state);

    result = detector.inCollision (state);
    EXPECT_FALSE (result);
}

void testCollisionDetector (const CollisionStrategy::Ptr& strategy)
{}

TEST (CollisionTest, mainCollisionTest)
{
    std::vector< CollisionStrategy::Ptr > strategies;
    for(std::string &s: rwlibs::proximitystrategies::ProximityStrategyFactory::getCollisionStrategyIDs()){
        strategies.push_back(rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy(s));
    }
    int idx = 0;

    for (const CollisionStrategy::Ptr& strategy : strategies) {
        testStrategy0 (strategy);

        testStrategy1 (strategy, idx);

        idx++;
    }

    EXPECT_FALSE(strategies.empty());

}
