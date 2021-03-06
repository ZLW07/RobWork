/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "LogStep.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rwsim::log;

LogStep::LogStep (SimulatorLogScope* parent) : SimulatorLogScope (parent)
{}

LogStep::~LogStep ()
{}

void LogStep::read (class InputArchive& iarchive, const std::string& id)
{
    iarchive.read (_interval.first, "IntervalFirst");
    iarchive.read (_interval.second, "IntervalSecond");
    SimulatorLogScope::read (iarchive, id);
}

void LogStep::write (class OutputArchive& oarchive, const std::string& id) const
{
    oarchive.write (_interval.first, "IntervalFirst");
    oarchive.write (_interval.second, "IntervalSecond");
    SimulatorLogScope::write (oarchive, id);
}

std::string LogStep::getType () const
{
    return "Step";
}

bool LogStep::operator== (const SimulatorLog& b) const
{
    if (const LogStep* const entry = dynamic_cast< const LogStep* > (&b)) {
        if (_interval != entry->_interval)
            return false;
    }
    return SimulatorLogScope::operator== (b);
}

std::string LogStep::getDescription () const
{
    std::stringstream str;
    str << "Step (time " << _interval.first << " to " << _interval.second << ")";
    return str.str ();
}

double LogStep::timeBegin () const
{
    return _interval.first;
}

double LogStep::timeEnd () const
{
    return _interval.second;
}

void LogStep::setTimeBegin (double time)
{
    _interval.first = time;
}

void LogStep::setTimeEnd (double time)
{
    _interval.second = time;
}
