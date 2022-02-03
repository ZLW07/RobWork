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

#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

#include "../TestEnvironment.hpp"

using namespace rwsim::dynamics;
using namespace rwsim::loaders;

TEST(CommonTest, DynamicWorkCellLoaderTest )
{
    // check failed loading of non-existing scene
	EXPECT_THROW( DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/devices/does_not_exist.dwc.xml"), std::exception);

	// check successfull loading of different dynamic workcells
    DynamicWorkCell::Ptr dwc_pg70, dwc_sdh, dwc_ur;
    EXPECT_NO_THROW( dwc_pg70 = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/devices/PG70/test_scene.dwc.xml") );
    EXPECT_NO_THROW( dwc_sdh = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/devices/SDH2/test_scene.dwc.xml") );
    EXPECT_NO_THROW( dwc_ur = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/devices/UR6855A/test_scene.dwc.xml") );

    // check that the scenes contain correct information
    EXPECT_FALSE(dwc_pg70->findController("GraspController").isNull());
    EXPECT_FALSE(dwc_pg70->findDevice("PG70").isNull());
    EXPECT_FALSE(dwc_pg70->findDevice<RigidDevice>("PG70").isNull());
    EXPECT_FALSE(dwc_pg70->findBody("PG70.Base").isNull());
    EXPECT_FALSE(dwc_pg70->findBody("PG70.RightFinger").isNull());

    // check different constraints
    DynamicWorkCell::Ptr fixed, prismatic, revolute, universal, spherical, piston, prismaticRotoid, prismaticUniversal, free, free_spring, revolute_limits;
    EXPECT_NO_THROW( fixed = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/fixed.dwc.xml") );
    EXPECT_NO_THROW( prismatic = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/prismatic.dwc.xml") );
    EXPECT_NO_THROW( revolute = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/revolute.dwc.xml") );
    EXPECT_NO_THROW( universal = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/universal.dwc.xml") );
    EXPECT_NO_THROW( spherical = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/spherical.dwc.xml") );
    EXPECT_NO_THROW( piston = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/piston.dwc.xml") );
    EXPECT_NO_THROW( prismaticRotoid = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/prismaticRotoid.dwc.xml") );
    EXPECT_NO_THROW( prismaticUniversal = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/prismaticUniversal.dwc.xml") );
    EXPECT_NO_THROW( free = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/free.dwc.xml") );
    EXPECT_NO_THROW( free_spring = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/free_spring.dwc.xml") );
    EXPECT_NO_THROW( revolute_limits = DynamicWorkCellLoader::load(TestEnvironment::testfilesDir() + "/scene/constraints/revolute_limits.dwc.xml") );

    // check information
    EXPECT_FALSE(fixed->findConstraint("Constraint").isNull());
    EXPECT_FALSE(prismatic->findConstraint("Constraint").isNull());
    EXPECT_FALSE(revolute->findConstraint("Constraint").isNull());
    EXPECT_FALSE(universal->findConstraint("Constraint").isNull());
    EXPECT_FALSE(spherical->findConstraint("Constraint").isNull());
    EXPECT_FALSE(piston->findConstraint("Constraint").isNull());
    EXPECT_FALSE(prismaticRotoid->findConstraint("Constraint").isNull());
    EXPECT_FALSE(prismaticUniversal->findConstraint("Constraint").isNull());
    EXPECT_FALSE(free->findConstraint("Constraint").isNull());
    EXPECT_FALSE(free_spring->findConstraint("Constraint").isNull());
    EXPECT_FALSE(revolute_limits->findConstraint("Constraint").isNull());

    EXPECT_FALSE(prismatic->findConstraint("Constraint")->getSpringParams().enabled);
    EXPECT_TRUE(free_spring->findConstraint("Constraint")->getSpringParams().enabled);
    EXPECT_FALSE(revolute->findConstraint("Constraint")->getLimit(0).lowOn);
    EXPECT_FALSE(revolute->findConstraint("Constraint")->getLimit(0).highOn);
    EXPECT_TRUE(revolute_limits->findConstraint("Constraint")->getLimit(0).lowOn);
    EXPECT_TRUE(revolute_limits->findConstraint("Constraint")->getLimit(0).highOn);
}

