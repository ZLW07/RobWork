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

/**
 * This file tests the force torque sensor
 */

#include <gtest/gtest.h>

#include <RobWorkSimConfig.hpp>

#include <rw/loaders/path/PathLoader.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

#include "../TestEnvironment.hpp"

using rw::core::ownedPtr;
using rw::kinematics::State;
using rw::math::Vector3D;
using namespace rw::trajectory;

using namespace rwsim::dynamics;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::sensor::SimulatedFTSensor;
using rwsim::simulator::ODESimulator;

TEST(FTSensorTest, FTSensor)
{
	// load a scene with a FT sensor mounted in between a kinematic body and a dynamic
	// place the kinematic body in different poses and pla
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( TestEnvironment::testfilesDir() + "/scene/sensors/single_object.dwc.xml" );

    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    KinematicBody::Ptr kbody = dwc->findBody<KinematicBody>("Floor");
    RigidBody::Ptr body = dwc->findBody<RigidBody>("Tray");

    SimulatedFTSensor::Ptr sensor = dwc->findSensor<SimulatedFTSensor>("FTSensor");
    TimedStatePath tpath;
    // test that the control interface works
    odesim->initPhysics(state);
    EXPECT_EQ(odesim->getTime(), 0.0 );
    for(int i=0; i<1000; i++){
    	//std::cout << i << "\t";
    	odesim->step(0.01, state);
    	// print the ft sensor readings
    	//Vector3D<> f = sensor->getForce(state);
    	//std::cout << odesim->getTime() << "\t" << f[0]<< "\t" << f[1]<< "\t" << f[2] << std::endl;
    	tpath.push_back( TimedState(odesim->getTime(),state));

    }
    rw::loaders::PathLoader::storeTimedStatePath(*dwc->getWorkcell(), tpath,"tpath.rwplay");



}