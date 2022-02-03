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

#include <RobWorkSimConfig.hpp>


#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

#if RWSIM_HAVE_ODE
#include <rwsimlibs/ode/ODEThreading.hpp>
using rwsim::simulator::ODEThreading;
#endif

using namespace rw::common;
using namespace rw::core;
using rw::kinematics::State;
using rw::models::WorkCell;
using rwlibs::task::GraspTask;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::simulator::GraspTaskSimulator;
using namespace rwsim::dynamics;

TEST(GraspTaskSimulatorTest, FullTest )
{
	std::string dwc_file = TestEnvironment::testfilesDir() + "/scene/grasping/pg70_box.dwc.xml";
	std::string grasptask_file = TestEnvironment::testfilesDir() + "/scene/grasping/grasptask_pg70_box.rwtask.xml";
	//rw::common::Log::getInstance()->setLevel( rw::common::Log::Debug );
    // add loading tests here
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwc_file);
    State initState = dwc->getWorkcell()->getDefaultState();
    WorkCell::Ptr wc = dwc->getWorkcell();

    // create GraspTaskSimulator
    GraspTaskSimulator::Ptr graspSim;
#if RWSIM_HAVE_ODE
    if (ODEThreading::isSupported())
        graspSim = ownedPtr( new GraspTaskSimulator(dwc, 2) );
    else
        graspSim = ownedPtr( new GraspTaskSimulator(dwc, 1) );
#else
    graspSim = ownedPtr( new GraspTaskSimulator(dwc, 2) );
#endif

    GraspTask::Ptr grasptask = GraspTask::load( grasptask_file );
    graspSim->setStoreTimedStatePaths(true);
    graspSim->forceSimulateAll(true);
    graspSim->load( grasptask );
    graspSim->startSimulation(initState);
    Timer timeoutTimer;
    do{
        TimerUtil::sleepMs(500);
        if(timeoutTimer.getTimeSec()>200){
        	EXPECT_LT(timeoutTimer.getTimeSec(),200);
        	break;
        }
    } while(graspSim->isRunning());

    // check outcome

    // merge and save all times states to get debug feedback
    typedef std::map<rwlibs::task::GraspSubTask*, std::map<rwlibs::task::GraspTarget*,rw::trajectory::TimedStatePath> > TStateMap;
    typedef std::map<rwlibs::task::GraspTarget*,rw::trajectory::TimedStatePath> TStateMap2;
    TStateMap paths = graspSim->getTimedStatePaths();

    rw::trajectory::TimedStatePath spath;
    double timeOffset = 0;
    for(TStateMap::value_type val : paths ) {
    	// take each timed state from the map
		for(TStateMap2::value_type val2 : val.second ) {
			for(rw::trajectory::TimedStatePath::size_type i = 0; i < val2.second.size(); i++) {
				spath.push_back( val2.second[i] );
				spath.back().getTime() = spath.back().getTime()+timeOffset;
			}
			// increase timeoffset
			timeOffset = spath.back().getTime();
		}
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc,spath,"spath-grasptasksim-test.rwplay");

    GraspTask::Ptr grasptask_result = graspSim->getResult();

    // check stuff in results

}
