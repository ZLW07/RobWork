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

#include "../TestEnvironment.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathplanners/prm/PartialIndexTable.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>

#include <gtest/gtest.h>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw::core;
using namespace rw::common;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using rw::proximity::CollisionStrategy;
using rw::trajectory::QPath;
using namespace rwlibs::pathplanners::prm;
using namespace rwlibs::pathplanners;

TEST (PathPlanning, testPartialIndexTable)
{
    Q lower = Q::zero (3);
    Q upper (3);
    upper (0) = upper (1) = upper (2) = 1;
    const double radius               = 0.1;
    Q weights (3);
    weights (0) = weights (1) = weights (2) = 1;
    const Device::QBox bounds (lower, upper);

    typedef PartialIndexTable< Q > Table;
    Table table (bounds, weights, radius, 3);

    QSampler::Ptr anyQ = QSampler::makeUniform (bounds);

    for (int i = 0; i < 1000; i++) {
        Q q = anyQ->sample ();
        table.addNode (q, q);
    }

    const Q pos = lower;

    // Now search for neighbors near pos.
    std::vector< Q > neighbors;
    table.searchNeighbors (pos, neighbors);

    QMetric::Ptr metric = MetricFactory::makeInfinity< Q > ();

    bool ok = true;
    for (const Q& q : neighbors) {
        const double dist = metric->distance (pos, q);

        const bool bad = dist > 2 * radius;
        if (bad) {
            std::cout << "PartialIndexTable::searchNeighbors(): Distance " << dist
                      << " to neighbor too great." << std::endl;
        }

        ok = ok && !bad;
    }

    EXPECT_TRUE (ok);
}

void testPathPlanning (const CollisionStrategy::Ptr& strategy)
{
    WorkCell::Ptr workcell =
        WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () + "simple/workcell.wc.xml");

    Device::Ptr device = workcell->findDevice ("PA10");
    const State& state = workcell->getDefaultState ();
    const PlannerConstraint constraint =
        PlannerConstraint::make (strategy, workcell, device, state);

    const Q from = device->getQ (state);
    bool res;

    ////////////////////// test local straight line planner
    QToQPlanner::Ptr line = QToQPlanner::make (constraint);
    // Plan a couple of straight-line paths.
    {
        Q linearToGood (7, 0.854336, 1.28309, -1.58383, -0.822832, -0.362664, -0.989248, -0.376991);
        Q linearToBad (7, 2.399, 1.28309, -1.58383, -0.822832, -0.362664, -0.989248, -0.376991);

        QPath path;
        res = line->query (from, linearToGood, path, 2);
        EXPECT_TRUE (res);
        EXPECT_TRUE (!PlannerUtil::inCollision (constraint, path));

        res = line->query (from, linearToBad, path, 2);
        EXPECT_TRUE (!res);

        // test wrong inputs
    }

    //////////////////// test global planners

    QToQPlanner::Ptr rrtplanner                  = RRTPlanner::makeQToQPlanner (constraint, device);
    QToQPlanner::Ptr arwplanner                  = ARWPlanner::makeQToQPlanner (constraint, device);
    QToQPlanner::Ptr sblplanner                  = SBLPlanner::makeQToQPlanner (SBLSetup::make (
        constraint.getQConstraintPtr (),
        QEdgeConstraintIncremental::makeDefault (constraint.getQConstraintPtr (), device),
        device));
    PRMPlanner::Ptr prmplanner_default           = ownedPtr (new PRMPlanner (
        constraint.getQConstraintPtr (), QSampler::makeUniform (*device), 0.01, *device, state));
    PRMPlanner::Ptr prmplanner_lazy_astar        = ownedPtr (new PRMPlanner (
        constraint.getQConstraintPtr (), QSampler::makeUniform (*device), 0.01, *device, state));
    PRMPlanner::Ptr prmplanner_lazy_dijkstra     = ownedPtr (new PRMPlanner (
        constraint.getQConstraintPtr (), QSampler::makeUniform (*device), 0.01, *device, state));
    PRMPlanner::Ptr prmplanner_lazy_brute_astar  = ownedPtr (new PRMPlanner (
        constraint.getQConstraintPtr (), QSampler::makeUniform (*device), 0.01, *device, state));
    PRMPlanner::Ptr prmplanner_lazy_kdtree_astar = ownedPtr (new PRMPlanner (
        constraint.getQConstraintPtr (), QSampler::makeUniform (*device), 0.01, *device, state));

    prmplanner_lazy_astar->setShortestPathSearchStrategy (PRMPlanner::A_STAR);
    prmplanner_lazy_astar->setNeighSearchStrategy (PRMPlanner::PARTIAL_INDEX_TABLE);

    prmplanner_lazy_dijkstra->setShortestPathSearchStrategy (PRMPlanner::DIJKSTRA);
    prmplanner_lazy_dijkstra->setNeighSearchStrategy (PRMPlanner::PARTIAL_INDEX_TABLE);

    prmplanner_lazy_brute_astar->setShortestPathSearchStrategy (PRMPlanner::A_STAR);
    prmplanner_lazy_brute_astar->setNeighSearchStrategy (PRMPlanner::BRUTE_FORCE);

    prmplanner_lazy_kdtree_astar->setShortestPathSearchStrategy (PRMPlanner::A_STAR);
    prmplanner_lazy_kdtree_astar->setNeighSearchStrategy (PRMPlanner::KDTREE);

    // building roadmaps
    prmplanner_default->buildRoadmap (1000);
    prmplanner_lazy_astar->buildRoadmap (1000);
    prmplanner_lazy_dijkstra->buildRoadmap (1000);
    prmplanner_lazy_brute_astar->buildRoadmap (1000);
    prmplanner_lazy_kdtree_astar->buildRoadmap (1000);

    // prmplanner->setCollisionCheckingStrategy(PRMPlanner::NODECHECK);
    // prmplanner->setShortestPathSearchStrategy(PRMPlanner::DIJKSTRA);
    // prmplanner->setNeighSearchStrategy(PRMPlanner::PARTIAL_INDEX_TABLE);
    // prmplanner->setAStarTimeOutTime(1);
    // prmplanner->buildRoadmap(2000);

    std::vector< std::pair< std::string, QToQPlanner::Ptr > > planners;
    planners.push_back (std::make_pair ("RRT", rrtplanner));
    planners.push_back (std::make_pair ("ARW", arwplanner));
    planners.push_back (std::make_pair ("SBL", sblplanner));
    planners.push_back (std::make_pair ("PRM (default)", prmplanner_default));
    planners.push_back (std::make_pair ("PRM (lazy,partial,astar)", prmplanner_lazy_astar));
    planners.push_back (std::make_pair ("PRM (lazy,partial,dijkstra)", prmplanner_lazy_dijkstra));
    planners.push_back (std::make_pair ("PRM (lazy,brute,astar)", prmplanner_lazy_brute_astar));
    planners.push_back (std::make_pair ("PRM (lazy,kdtree,astar)", prmplanner_lazy_kdtree_astar));

    // Plan some paths to random configurations with RRT and SBL.
    {
        // first generate targets to plan to
        QSampler::Ptr cfreeQ = QSampler::makeConstrained (
            QSampler::makeUniform (device), constraint.getQConstraintPtr (), 1000);

        std::vector< Q > samples;
        for (int i = 0; i < 10; i++) {
            samples.push_back (cfreeQ->sample ());
        }

        // next test on each planner
        typedef std::pair< std::string, QToQPlanner::Ptr > Pair;
        for (const Pair planner : planners) {
            std::cout << "Testing QToQPlanner " << planner.first;
            Timer time;
            for (const Q to : samples) {
                QPath path;

                res = planner.second->query (from, to, path, 20);
                EXPECT_TRUE (res);
                EXPECT_TRUE (!PlannerUtil::inCollision (constraint, path));
            }
            std::cout << " time:" << time.getTimeMs () << "ms" << std::endl;
        }
    }
}

TEST (PathPlanning, testPathPlanningMain)
{
    CollisionStrategy::Ptr strategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy();
    testPathPlanning (strategy);
}
