/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#include "../TestEnvironment.hpp"

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Random.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.cpp>

#include <gtest/gtest.h>

#if !defined(__GNUC__) || BOOST_VERSION >= 106400 || __GNUC__ >= 7 ||                   \
    (__GNUC__ == 6 && __GNUC_MINOR__ >= 1) || (__GNUC__ == 5 && __GNUC_MINOR__ >= 4) || \
    (__GNUC__ == 4 && __GNUC_MINOR__ >= 9 && __GNUC_PATCHLEVEL__ >= 4)
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <sstream>
#endif

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::trajectory;
using rw::trajectory::QPath;

TEST (BoostSerialization, Path)
{
    QPath path (5);

    for (std::size_t i = 0; i < path.size (); ++i) {
        std::size_t size = Random::ranI (2, 15);
        path[i]          = Q (size);

        for (std::size_t k = 0; k < size; ++k) {
            path[i][k] = Random::ran (-10, 10);
        }
    }
    EXPECT_EQ (std::size_t (5), path.size ());

#if !defined(__GNUC__) || BOOST_VERSION >= 106400 || __GNUC__ >= 7 ||                   \
    (__GNUC__ == 6 && __GNUC_MINOR__ >= 1) || (__GNUC__ == 5 && __GNUC_MINOR__ >= 4) || \
    (__GNUC__ == 4 && __GNUC_MINOR__ >= 9 && __GNUC_PATCHLEVEL__ >= 4)
    std::stringstream stream;
    boost::archive::text_oarchive oa (stream);
    oa << path;

    // Use boost to load class data
    boost::archive::text_iarchive ia (stream);
    QPath loadedpath;
    ia >> loadedpath;

    EXPECT_EQ (path.size (), loadedpath.size ());
    for (std::size_t i = 0; i < std::min (path.size (), loadedpath.size ()); ++i) {
        EXPECT_EQ (path[i], loadedpath[i]);
    }
#endif
}
TEST (PathTest, simpleQPath)
{
    QPath path;
    Q q (7);
    q (0) = 1;
    path.push_back (q);
    q (0) = 2;
    path.push_back (q);
    q (0) = 3;
    path.push_back (q);
    int index = 1;
    for (Q q : path) {
        EXPECT_EQ (q (0), index);
        index++;
    }
}

TEST (PathTest, simpleTransform3DPath)
{
    Transform3DPath path;
    Transform3D<> t;
    t (0, 0) = 1;
    path.push_back (t);
    t (0, 0) = 2;
    path.push_back (t);
    t (0, 0) = 3;
    path.push_back (t);

    int index = 1;
    for (Transform3D< double > t : path) {
        EXPECT_EQ (t (0, 0), index);
        index++;
    }
}
TEST (PathTest, simpleStatePath)
{
    StatePath statepath;
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                            "MultiRobotDemo/Scene.wc.xml");
    EXPECT_FALSE (workcell.isNull ());
    Device::Ptr dev      = workcell->getDevices ().front ();
    const State defstate = workcell->getDefaultState ();

    // Test StatePath and the possibility of creating
    {
        StatePath statepath;
        State state = defstate;

        statepath.push_back (state);
        Q q = dev->getQ (state);
        for (size_t i = 0; i < q.size (); i++)
            q (i) = 0.5;
        dev->setQ (q, state);
        statepath.push_back (state);
        for (size_t i = 0; i < q.size (); i++)
            q (i) = -1;
        dev->setQ (q, state);
        statepath.push_back (state);

        StateTrajectory::Ptr statetrajectory =
            TrajectoryFactory::makeLinearTrajectoryUnitStep (statepath);
        State s0 = statetrajectory->x (0);
        q        = dev->getQ (s0);
        EXPECT_EQ (q (0), 0);

        State s05 = statetrajectory->x (0.5);
        q         = dev->getQ (s05);
        EXPECT_EQ (q (0), 0.25);

        State s1 = statetrajectory->x (1);
        q        = dev->getQ (s1);
        EXPECT_EQ (q (0), 0.5);

        State s15 = statetrajectory->x (1.5);
        q         = dev->getQ (s15);
        std::cout << "q = " << q << std::endl;
        EXPECT_EQ (q (0), -0.25);

        State s2 = statetrajectory->x (2);
        q        = dev->getQ (s2);
        EXPECT_EQ (q (0), -1);
    }

    // Test StatePath and the possibility of creating
    {
        TimedStatePath timedStatePath;
        WorkCell::Ptr workcell = WorkCellLoader::Factory::load (TestEnvironment::testfilesDir () +
                                                                "MultiRobotDemo/Scene.wc.xml");
        EXPECT_FALSE (workcell.isNull ());

        Device::Ptr dev = workcell->getDevices ().front ();
        State state     = workcell->getDefaultState ();
        timedStatePath.push_back (Timed< State > (0, state));

        Q q = dev->getQ (state);
        for (size_t i = 0; i < q.size (); i++)
            q (i) = 0.5;
        dev->setQ (q, state);
        timedStatePath.push_back (Timed< State > (12, state));

        for (size_t i = 0; i < q.size (); i++)
            q (i) = -1;
        dev->setQ (q, state);
        timedStatePath.push_back (Timed< State > (24, state));

        StateTrajectory::Ptr statetrajectory =
            TrajectoryFactory::makeLinearTrajectory (timedStatePath);
        State s0 = statetrajectory->x (0);
        q        = dev->getQ (s0);
        std::cout << "q(0) = " << q << std::endl;
        EXPECT_EQ (q (0), 0);

        State s3 = statetrajectory->x (3);
        q        = dev->getQ (s3);
        std::cout << "q(3) = " << q << std::endl;
        EXPECT_EQ (q (0), 0.125);

        State s1 = statetrajectory->x (12);
        q        = dev->getQ (s1);
        EXPECT_EQ (q (0), 0.5);

        State s15 = statetrajectory->x (18);
        q         = dev->getQ (s15);
        std::cout << "q(18) = " << q << std::endl;
        EXPECT_EQ (q (0), -0.25);

        State s2 = statetrajectory->x (24);
        q        = dev->getQ (s2);
        EXPECT_EQ (q (0), -1);
    }
}
