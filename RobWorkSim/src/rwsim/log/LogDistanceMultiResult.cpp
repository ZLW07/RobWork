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

#include "LogDistanceMultiResult.hpp"

#include "LogPositions.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rwsim::log;

LogDistanceMultiResult::LogDistanceMultiResult (SimulatorLogScope* parent) :
    SimulatorLogEntry (parent)
{}

LogDistanceMultiResult::~LogDistanceMultiResult ()
{}

void LogDistanceMultiResult::read (class InputArchive& iarchive, const std::string& id)
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
        const unsigned int nr = iarchive.readUInt ("Distances");
        info.result.p1s.resize (nr);
        info.result.p2s.resize (nr);
        info.result.geoIdxA.resize (nr);
        info.result.geoIdxB.resize (nr);
        info.result.p1prims.resize (nr);
        info.result.p2prims.resize (nr);
        info.result.distances.resize (nr);
        for (unsigned int i = 0; i < nr; i++) {
            iarchive.read (info.result.p1s[i], "ClosestPoint1");
            iarchive.read (info.result.p2s[i], "ClosestPoint2");
            iarchive.read (info.result.geoIdxA[i], "GeometryIndexA");
            iarchive.read (info.result.geoIdxB[i], "GeometryIndexB");
            iarchive.read (info.result.p1prims[i], "FaceIndexA");
            iarchive.read (info.result.p2prims[i], "FaceIndexB");
            iarchive.read (info.result.distances[i], "Distance");
        }
    }
    SimulatorLogEntry::read (iarchive, id);
}

void LogDistanceMultiResult::write (class OutputArchive& oarchive, const std::string& id) const
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
        oarchive.write (info.result.distances.size (), "Distances");
        for (std::size_t i = 0; i < info.result.distances.size (); i++) {
            oarchive.write (info.result.p1s[i], "ClosestPoint1");
            oarchive.write (info.result.p2s[i], "ClosestPoint2");
            oarchive.write (info.result.geoIdxA[i], "GeometryIndexA");
            oarchive.write (info.result.geoIdxB[i], "GeometryIndexB");
            oarchive.write (info.result.p1prims[i], "FaceIndexA");
            oarchive.write (info.result.p2prims[i], "FaceIndexB");
            oarchive.write (info.result.distances[i], "Distance");
        }
    }
    SimulatorLogEntry::write (oarchive, id);
}

std::string LogDistanceMultiResult::getType () const
{
    return getTypeID ();
}

bool LogDistanceMultiResult::ResultInfo::operator== (const ResultInfo& b) const
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
    if (result.p1s != b.result.p1s)
        return false;
    if (result.p2s != b.result.p2s)
        return false;
    if (result.geoIdxA != b.result.geoIdxA)
        return false;
    if (result.geoIdxB != b.result.geoIdxB)
        return false;
    if (result.p1prims != b.result.p1prims)
        return false;
    if (result.p2prims != b.result.p2prims)
        return false;
    if (result.distances != b.result.distances)
        return false;

    return true;
}

bool LogDistanceMultiResult::operator== (const SimulatorLog& b) const
{
    if (const LogDistanceMultiResult* const entry =
            dynamic_cast< const LogDistanceMultiResult* > (&b)) {
        if (*_positions != *entry->_positions)
            return false;
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

std::list< SimulatorLogEntry::Ptr > LogDistanceMultiResult::getLinkedEntries () const
{
    if (_positions == NULL)
        return std::list< SimulatorLogEntry::Ptr > ();
    else
        return std::list< SimulatorLogEntry::Ptr > (1, _positions);
}

bool LogDistanceMultiResult::autoLink ()
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

SimulatorLogEntry::Ptr LogDistanceMultiResult::createNew (SimulatorLogScope* parent) const
{
    return ownedPtr (new LogDistanceMultiResult (parent));
}

std::string LogDistanceMultiResult::getTypeID ()
{
    return "DistanceMultiResult";
}

const std::vector< LogDistanceMultiResult::ResultInfo >& LogDistanceMultiResult::getResults () const
{
    return _results;
}

void LogDistanceMultiResult::addResult (const DistanceMultiStrategy::Result& result)
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

void LogDistanceMultiResult::addResults (
    const std::vector< DistanceMultiStrategy::Result >& results)
{
    for (const DistanceMultiStrategy::Result& result : results) {
        addResult (result);
    }
}

LogPositions::Ptr LogDistanceMultiResult::getPositions () const
{
    return _positions;
}
