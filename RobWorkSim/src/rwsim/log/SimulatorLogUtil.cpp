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

#include "SimulatorLogUtil.hpp"

#include "LogCollisionResult.hpp"
#include "LogDistanceMultiResult.hpp"
#include "LogDistanceResult.hpp"
#include "LogMessage.hpp"
#include "LogPositions.hpp"
#include "LogStep.hpp"
#include "LogValues.hpp"
#include "LogVelocities.hpp"
#include "SimulatorLogScope.hpp"

using rw::core::ownedPtr;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::log;

SimulatorLogUtil::SimulatorLogUtil () : _scope (NULL)
{}

SimulatorLogUtil::~SimulatorLogUtil ()
{}

void SimulatorLogUtil::setSimulatorLog (SimulatorLogScope::Ptr log)
{
    _log   = log;
    _scope = log.get ();
}

bool SimulatorLogUtil::doLog () const
{
    return _log != NULL;
}

void SimulatorLogUtil::beginStep (double time, const char* file, int line)
{
    if (!doLog ())
        return;
    LogStep::Ptr step = ownedPtr (new LogStep (_scope));
    _scope->appendChild (step);
    step->setTimeBegin (time);
    step->setFilename (file);
    step->setLineBegin (line);
    _scope = step.get ();
}

void SimulatorLogUtil::endStep (double time, int line)
{
    if (!doLog ())
        return;
    LogStep* const step = dynamic_cast< LogStep* > (_scope);
    if (step == NULL)
        RW_THROW ("Could not end step! - Not in correct scope.");
    step->setTimeEnd (time);
    step->setLineEnd (line);
    _scope = step->getParent ();
}

void SimulatorLogUtil::beginSection (const std::string& name, const char* file, int line)
{
    if (!doLog ())
        return;
    SimulatorLogScope::Ptr section = ownedPtr (new SimulatorLogScope (_scope));
    _scope->appendChild (section);
    section->setDescription (name);
    section->setFilename (file);
    section->setLineBegin (line);
    _scope = section.get ();
}

void SimulatorLogUtil::endSection (int line)
{
    if (!doLog ())
        return;
    SimulatorLogScope* const section = dynamic_cast< SimulatorLogScope* > (_scope);
    if (section == NULL)
        RW_THROW ("Could not end section! - Not in correct scope.");
    section->setLineEnd (line);
    _scope = section->getParent ();
}

SimulatorLogScope* SimulatorLogUtil::makeScope (const std::string& name, const char* file, int line)
{
    if (!doLog ())
        return NULL;
    SimulatorLogScope::Ptr section = ownedPtr (new SimulatorLogScope (_scope));
    _scope->appendChild (section);
    section->setDescription (name);
    section->setFilename (file);
    section->setLineBegin (line);
    section->setLineEnd (line);
    return section.get ();
}

void SimulatorLogUtil::addValues (const std::string& description,
                                  const std::vector< double >& values,
                                  const std::vector< std::string >& labels, const char* file,
                                  int line)
{
    if (!doLog ())
        return;
    LogValues::Ptr entry = ownedPtr (new LogValues (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->setData (labels, values);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}

std::ostream& SimulatorLogUtil::log (const std::string& description, const char* file, int line)
{
    if (!doLog ()) {
        _dummyStream.str (std::string ());
        return _dummyStream;
    }
    LogMessage::Ptr entry = ownedPtr (new LogMessage (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
    return entry->stream ();
}

std::ostream& SimulatorLogUtil::log (const char* file, int line)
{
    if (!doLog ()) {
        _dummyStream.str (std::string ());
        return _dummyStream;
    }
    LogMessage::Ptr entry = ownedPtr (new LogMessage (_scope));
    _scope->appendChild (entry);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
    return entry->stream ();
}

void SimulatorLogUtil::addPositions (const std::string& description,
                                     const std::map< std::string, Transform3D<> >& positions,
                                     const char* file, int line)
{
    if (!doLog ())
        return;
    LogPositions::Ptr entry = ownedPtr (new LogPositions (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->setPositions (positions);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}

void SimulatorLogUtil::addVelocities (const std::string& description,
                                      const std::map< std::string, VelocityScrew6D<> >& velocities,
                                      const char* file, int line)
{
    if (!doLog ())
        return;
    LogVelocities::Ptr entry = ownedPtr (new LogVelocities (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->setVelocities (velocities);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}

void SimulatorLogUtil::addCollisionResults (const std::string& description,
                                            const std::vector< CollisionStrategy::Result >& results,
                                            const char* file, int line)
{
    if (!doLog ())
        return;
    LogCollisionResult::Ptr entry = ownedPtr (new LogCollisionResult (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->addResults (results);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}

void SimulatorLogUtil::addDistanceResults (const std::string& description,
                                           const std::vector< DistanceStrategy::Result >& results,
                                           const char* file, int line)
{
    if (!doLog ())
        return;
    LogDistanceResult::Ptr entry = ownedPtr (new LogDistanceResult (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->addResults (results);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}

void SimulatorLogUtil::addDistanceMultiResults (
    const std::string& description, const std::vector< DistanceMultiStrategy::Result >& results,
    const char* file, int line)
{
    if (!doLog ())
        return;
    LogDistanceMultiResult::Ptr entry = ownedPtr (new LogDistanceMultiResult (_scope));
    _scope->appendChild (entry);
    entry->setDescription (description);
    entry->addResults (results);
    entry->setFilename (file);
    entry->setLine (line);
    entry->autoLink ();
}
