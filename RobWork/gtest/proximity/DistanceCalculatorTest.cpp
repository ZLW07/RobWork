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
#include <rw/proximity/DistanceCalculator.hpp>

#include <gtest/gtest.h>
#include <string>
#include <vector>
//#include <iostream>

using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace rw::math;
using namespace std;

namespace {
class DistanceCalculatorTest : public ::testing::TestWithParam< std::string >
{
  public:
    const string workcellFile = TestEnvironment::testfilesDir () + "/workcells/DistanceTest.wc.xml";
    rw::models::WorkCell::Ptr wc;
    rw::kinematics::State initialState;

    DistanceStrategy::Ptr strategy;
    DistanceCalculator::Ptr distCalc;

    virtual void SetUp ()
    {
        wc           = WorkCellLoader::Factory::load (workcellFile);
        initialState = wc->getDefaultState ();

        strategy = DistanceStrategy::Factory::makeStrategy (GetParam ());
        strategy->clear ();
        ASSERT_FALSE (strategy.isNull ());
        distCalc = new DistanceCalculator (wc, strategy);
    }

    void TearDown ()
    {
        // Important: models must be destroyed as some strategies must be able to cleanup internal
        // caches. PQP models are for instance cached based on the memory address of the
        // GeometryData object (other tests might create a different geometry on the same address)
        if (!strategy.isNull ()) {
            strategy->clearFrames ();
            strategy->clear ();
        }
    }
};
std::vector< std::string > strategies = DistanceStrategy::Factory::getStrategies ();
}    // namespace

INSTANTIATE_TEST_CASE_P (DistanceCalculator, DistanceCalculatorTest,
                         ::testing::ValuesIn (strategies));

TEST_P (DistanceCalculatorTest, SimpleGeometry)
{
    vector< DistanceStrategy::Result >* distances = new vector< DistanceStrategy::Result >;

    // distances vector should be empty:
    EXPECT_EQ (distances->size (), 0u);

    distCalc->distance (initialState, distances);

    EXPECT_EQ (distances->size (), 6u);

    // Distance from Sphere1 - Box1
    //  distance = center_distance{1} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR (distances->at (0).distance, 0.7, 0.0001);

    // Distance from sphere1 to sphere2:
    //  distance = center_dist{sqrt(1²+2²+2²)}- 2* radius{0.2}
    EXPECT_NEAR (distances->at (1).distance, 2.6, 0.001);

    // Distance from sphere1 to box2:
    //  distance = center_distance{2} - radius{0.2} - side_length{0.2}/2
    EXPECT_NEAR (distances->at (2).distance, 1.7, 0.001);

    // Distance from box1 to box2:
    //  distance = center_distance{2-1}-2*side_length{0.2}/2
    EXPECT_NEAR (distances->at (4).distance, 0.8, 0.001);
}