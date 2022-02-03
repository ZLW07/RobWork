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

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/proximity/ProximityModel.hpp>
#include <rw/proximity/ProximityStrategy.hpp>
#include <rw/core/Ptr.hpp>

#include <gtest/gtest.h>

using rw::core::ownedPtr;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;

namespace {
class ProximityModelTest : public ::testing::TestWithParam< std::string >
{
  protected:
    void SetUp ()
    {
        strategy = ProximityStrategy::Factory::makeStrategy (GetParam ());
        ASSERT_FALSE (strategy.isNull ());

        frames.push_back (new MovableFrame ("Frame1"));
        frames.push_back (new MovableFrame ("Frame2"));
        frames.push_back (new MovableFrame ("Frame3"));

        sstruct.addFrame (frames[0]);
        sstruct.addFrame (frames[1]);
        sstruct.addFrame (frames[2]);

        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo1")));
        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo2")));
        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo3")));
        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo4")));
        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo5")));
        geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "Geo6")));
        geoms[0]->setFrame (frames[0]);
        geoms[1]->setFrame (frames[0]);
        geoms[2]->setFrame (frames[0]);
        geoms[3]->setFrame (frames[1]);
        geoms[4]->setFrame (frames[1]);
        geoms[5]->setFrame (frames[2]);

        strategy->clearFrames ();

        for (size_t i = 0; i < geoms.size (); i++) {
            strategy->addModel (geoms[i]->getFrame (), geoms[i]);
        }

        for (size_t i = 0; i < 3; i++) {
            proxModels.push_back (strategy->getModel (frames[i]));
        }
    }

    ProximityStrategy::Ptr strategy;
    std::vector< Frame* > frames;
    StateStructure sstruct;
    std::vector< Geometry::Ptr > geoms;
    std::vector< ProximityModel::Ptr > proxModels;
};
std::vector< std::string > strategies = ProximityStrategy::Factory::getStrategies ();
}    // namespace

INSTANTIATE_TEST_CASE_P (ProximityModel, ProximityModelTest, ::testing::ValuesIn (strategies));

TEST_P (ProximityModelTest, getFrame)
{
    EXPECT_EQ (proxModels.size (), 3u);
    EXPECT_FALSE (proxModels[0].isNull ());
    EXPECT_FALSE (proxModels[0]->getFrame () == NULL);
    EXPECT_EQ(proxModels[0]->getFrame ()->getName(),"Frame1");

    EXPECT_FALSE (proxModels[1].isNull ());
    EXPECT_FALSE (proxModels[1]->getFrame () == NULL);
    EXPECT_EQ(proxModels[1]->getFrame ()->getName(),"Frame2");

    EXPECT_FALSE (proxModels[2].isNull ());
    EXPECT_FALSE (proxModels[2]->getFrame () == NULL);
    EXPECT_EQ(proxModels[2]->getFrame ()->getName(),"Frame3");
}

TEST_P (ProximityModelTest, addRemoveGeometry)
{
    geoms.push_back (ownedPtr (new Geometry (ownedPtr (new Sphere (1)), "GeoT1")));

    EXPECT_TRUE (proxModels[0]->addGeometry (geoms.back ()));
    std::vector< std::string > geoIDs = proxModels[0]->getGeometryIDs ();
    EXPECT_EQ (geoIDs.size (), 4u);
    EXPECT_EQ (geoIDs[0], "Geo1");
    EXPECT_EQ (geoIDs[1], "Geo2");
    EXPECT_EQ (geoIDs[2], "Geo3");
    EXPECT_EQ (geoIDs[3], "GeoT1");

    std::vector< Geometry::Ptr > gosList = proxModels[0]->getGeometries ();

    EXPECT_EQ (gosList.size (), 4u);
    EXPECT_EQ (gosList[0]->getId(), "Geo1");
    EXPECT_EQ (gosList[1]->getId(), "Geo2");
    EXPECT_EQ (gosList[2]->getId(), "Geo3");
    EXPECT_EQ (gosList[3]->getId(), "GeoT1");

    EXPECT_TRUE (proxModels[0]->removeGeometry("Geo2"));

    gosList = proxModels[0]->getGeometries ();

    EXPECT_EQ (gosList.size (), 3u);
    EXPECT_EQ (gosList[0]->getId(), "Geo1");
    EXPECT_EQ (gosList[1]->getId(), "Geo3");
    EXPECT_EQ (gosList[2]->getId(), "GeoT1");

    EXPECT_EQ(gosList[0],geoms[0]);
    EXPECT_EQ(gosList[1],geoms[2]);
    EXPECT_EQ(gosList[2],geoms.back());
}
