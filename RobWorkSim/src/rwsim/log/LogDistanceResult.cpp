/********************************************************************************
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
 ********************************************************************************/

#include "LogDistanceResult.hpp"

#include "LogPositions.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rwsim::log;

LogDistanceResult::LogDistanceResult (SimulatorLogScope* parent) : SimulatorLogEntry (parent)
{}

LogDistanceResult::~LogDistanceResult ()
{}

void LogDistanceResult::read (class InputArchive& iarchive, const std::string& id)
{
    _results.clear ();
    unsigned int n;
    n = iarchive.readUInt ("Results");
    _results.resize (n);
    for (ResultInfo& info : _results) {
        info.frameA = iarchive.readString ("FrameA");
        info.frameB = iarchive.readString ("FrameB");
        iarchive.read (info.geoNamesA, "GeometryIDsA");
        iarchive.read (info.geoNamesB, "GeometryIDsB");
        iarchive.read (info.result.p1, "ClosestPoint1");
        iarchive.read (info.result.p2, "ClosestPoint2");
        iarchive.read (info.result.distance, "Distance");
        iarchive.read (info.result.geoIdxA, "GeometryIndexA");
        iarchive.read (info.result.geoIdxB, "GeometryIndexB");
        iarchive.read (info.result.idx1, "FaceIndexA");
        iarchive.read (info.result.idx2, "FaceIndexB");
    }
    SimulatorLogEntry::read (iarchive, id);
}

void LogDistanceResult::write (class OutputArchive& oarchive, const std::string& id) const
{
    oarchive.write (_results.size (), "Results");
    for (const ResultInfo& info : _results) {
        oarchive.write (info.frameA, "FrameA");
        oarchive.write (info.frameB, "FrameB");
        oarchive.write (info.geoNamesA, "GeometryIDsA");
        oarchive.write (info.geoNamesB, "GeometryIDsB");
        oarchive.write (info.result.p1, "ClosestPoint1");
        oarchive.write (info.result.p2, "ClosestPoint2");
        oarchive.write (info.result.distance, "Distance");
        oarchive.write (info.result.geoIdxA, "GeometryIndexA");
        oarchive.write (info.result.geoIdxB, "GeometryIndexB");
        oarchive.write (info.result.idx1, "FaceIndexA");
        oarchive.write (info.result.idx2, "FaceIndexB");
    }
    SimulatorLogEntry::write (oarchive, id);
}

std::string LogDistanceResult::getType () const
{
    return getTypeID ();
}

bool LogDistanceResult::ResultInfo::operator== (const ResultInfo& b) const
{
    if (frameA != b.frameA)
        return false;
    if (frameB != b.frameB)
        return false;
    if (geoNamesA != b.geoNamesA)
        return false;
    if (geoNamesB != b.geoNamesB)
        return false;

    if (result.p1 != b.result.p1)
        return false;
    if (result.p2 != b.result.p2)
        return false;
    if (result.distance != b.result.distance)
        return false;
    if (result.geoIdxA != b.result.geoIdxA)
        return false;
    if (result.geoIdxB != b.result.geoIdxB)
        return false;
    if (result.idx1 != b.result.idx1)
        return false;
    if (result.idx2 != b.result.idx2)
        return false;

    return true;
}

bool LogDistanceResult::operator== (const SimulatorLog& b) const
{
    if (const LogDistanceResult* const entry = dynamic_cast< const LogDistanceResult* > (&b)) {
        if (*_positions != *entry->_positions)
            return false;
        // if (_results != entry->_results)
        //  return false;
        std::list< ResultInfo > unmatched;
        unmatched.insert (unmatched.end (), _results.begin (), _results.end ());
        for (std::size_t i = 0; i < entry->_results.size (); i++) {
            std::list< ResultInfo >::iterator it;
            bool found = false;
            for (it = unmatched.begin (); it != unmatched.end (); it++) {
                if (entry->_results[i] == *it) {
                    found = true;
                    unmatched.erase (it);
                    break;
                }
            }
            if (!found)
                return false;
        }
    }
    return SimulatorLogEntry::operator== (b);
}

std::list< SimulatorLogEntry::Ptr > LogDistanceResult::getLinkedEntries () const
{
    if (_positions == NULL)
        return std::list< SimulatorLogEntry::Ptr > ();
    else
        return std::list< SimulatorLogEntry::Ptr > (1, _positions);
}

bool LogDistanceResult::autoLink ()
{
    _positions = NULL;
    // Link to last position entry in tree
    SimulatorLogScope* scope = getParent ();
    if (scope == NULL)
        return false;
    SimulatorLog* find = this;
    bool found         = false;
    while (scope != NULL && !found) {
        // Find position of entry
        const std::vector< SimulatorLog::Ptr > children = scope->getChildren ();
        std::vector< SimulatorLog::Ptr >::const_reverse_iterator it;
        for (it = children.rbegin (); it != children.rend (); it++) {
            if (it->isNull ())
                continue;
            if (it->get () == find) {
                break;
            }
        }
        if (it != children.rend ()) {
            if (it->get () == find)
                it++;
        }
        // Now search backwards
        for (; it != children.rend (); it++) {
            RW_ASSERT (*it != NULL);
            _positions = (*it).cast< LogPositions > ();
            if (_positions != NULL) {
                found = true;
                break;
            }
        }
        find  = scope;
        scope = scope->getParent ();
    }
    return _positions != NULL;
}

SimulatorLogEntry::Ptr LogDistanceResult::createNew (SimulatorLogScope* parent) const
{
    return ownedPtr (new LogDistanceResult (parent));
}

std::string LogDistanceResult::getTypeID ()
{
    return "DistanceResult";
}

const std::vector< LogDistanceResult::ResultInfo >& LogDistanceResult::getResults () const
{
    return _results;
}

void LogDistanceResult::addResult (const DistanceStrategy::Result& result)
{
    using rw::kinematics::Frame;

    _results.resize (_results.size () + 1);
    ResultInfo& info            = _results.back ();
    info.result                 = result;
    const ProximityModel::Ptr a = result.a;
    const ProximityModel::Ptr b = result.b;
    if (a != NULL) {
        const Frame* const frame = a->getFrame ();
        if (frame != NULL)
            info.frameA = frame->getName ();
        info.geoNamesA = a->getGeometryIDs ();
        info.result.a  = NULL;
    }
    if (b != NULL) {
        const Frame* const frame = b->getFrame ();
        if (frame != NULL)
            info.frameB = frame->getName ();
        info.geoNamesB = b->getGeometryIDs ();
        info.result.b  = NULL;
    }
}

void LogDistanceResult::addResults (const std::vector< DistanceStrategy::Result >& results)
{
    for (const DistanceStrategy::Result& result : results) {
        addResult (result);
    }
}

LogPositions::Ptr LogDistanceResult::getPositions () const
{
    return _positions;
}
