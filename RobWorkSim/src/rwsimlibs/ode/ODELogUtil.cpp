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

#include "ODELogUtil.hpp"

#include "ODEBody.hpp"

using rw::kinematics::State;
using namespace rw::math;
using rwsim::simulator::ODEBody;
using rwsim::simulator::ODELogUtil;

ODELogUtil::ODELogUtil ()
{}

ODELogUtil::~ODELogUtil ()
{}

void ODELogUtil::addPositions (const std::string& description,
                               const std::map< std::string, rw::math::Transform3D<> >& positions,
                               const char* file, int line)
{
    SimulatorLogUtil::addPositions (description, positions, file, line);
}

void ODELogUtil::addPositions (const std::string& description,
                               const std::vector< ODEBody* >& bodies, const State& state,
                               const char* file, int line)
{
    if (!doLog ())
        return;
    std::map< std::string, Transform3D<> > pos;
    for (const ODEBody* const body : bodies) {
        pos[body->getRwBody ()->getName ()] = body->getRwBody ()->wTbf (state);
    }
    addPositions (description, pos, file, line);
}

void ODELogUtil::addVelocities (
    const std::string& description,
    const std::map< std::string, rw::math::VelocityScrew6D<> >& velocities, const char* file,
    int line)
{
    SimulatorLogUtil::addVelocities (description, velocities, file, line);
}

void ODELogUtil::addVelocities (const std::string& description,
                                const std::vector< ODEBody* >& bodies, const State& state,
                                const char* file, int line)
{
    if (!doLog ())
        return;
    std::map< std::string, VelocityScrew6D<> > vel;
    for (const ODEBody* const body : bodies) {
        vel[body->getRwBody ()->getName ()] = body->getRwBody ()->getVelocity (state);
    }
    addVelocities (description, vel, file, line);
}
