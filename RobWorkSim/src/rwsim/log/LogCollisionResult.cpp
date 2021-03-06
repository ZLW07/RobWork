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

#include "LogCollisionResult.hpp"

#include "LogPositions.hpp"
#include "SimulatorLogScope.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwsim::log;

LogCollisionResult::LogCollisionResult (SimulatorLogScope* parent) : SimulatorLogEntry (parent)
{}

LogCollisionResult::~LogCollisionResult ()
{}

void LogCollisionResult::read (class InputArchive& iarchive, const std::string& id)
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
        iarchive.read (info.result._aTb, "aTb");
        n = iarchive.readUInt ("CollisionPairs");
        info.result._collisionPairs.resize (n);
        for (CollisionStrategy::Result::CollisionPair& pair : info.result._collisionPairs) {
            pair.geoIdxA  = iarchive.readInt ("GeoIdxA");
            pair.geoIdxB  = iarchive.readInt ("GeoIdxB");
            pair.startIdx = iarchive.readInt ("StartIdx");
            pair.size     = iarchive.readInt ("Size");
        }
        n = iarchive.readUInt ("GeomPrimIds");
        info.result._geomPrimIds.resize (n);
        typedef std::pair< int, int > IntPair;
        for (IntPair& pair : info.result._geomPrimIds) {
            pair.first  = iarchive.readInt ("First");
            pair.second = iarchive.readInt ("Second");
        }
        info.result._nrBVTests   = iarchive.readInt ("NrBVTests");
        info.result._nrPrimTests = iarchive.readInt ("NrPrimTests");
    }
    SimulatorLogEntry::read (iarchive, id);
}

void LogCollisionResult::write (class OutputArchive& oarchive, const std::string& id) const
{
    oarchive.write (_results.size (), "Results");
    for (const ResultInfo& info : _results) {
        oarchive.write (info.frameA, "FrameA");
        oarchive.write (info.frameB, "FrameB");
        oarchive.write (info.geoNamesA, "GeometryIDsA");
        oarchive.write (info.geoNamesB, "GeometryIDsB");
        oarchive.write (info.result._aTb, "aTb");
        oarchive.write (info.result._collisionPairs.size (), "CollisionPairs");
        for (const CollisionStrategy::Result::CollisionPair& pair : info.result._collisionPairs) {
            oarchive.write (pair.geoIdxA, "GeoIdxA");
            oarchive.write (pair.geoIdxB, "GeoIdxB");
            oarchive.write (pair.startIdx, "StartIdx");
            oarchive.write (pair.size, "Size");
        }
        oarchive.write (info.result._geomPrimIds.size (), "GeomPrimIds");
        typedef std::pair< int, int > IntPair;
        for (const IntPair& pair : info.result._geomPrimIds) {
            oarchive.write (pair.first, "First");
            oarchive.write (pair.second, "Second");
        }
        oarchive.write (info.result._nrBVTests, "NrBVTests");
        oarchive.write (info.result._nrPrimTests, "NrPrimTests");
    }
    SimulatorLogEntry::write (oarchive, id);
}

std::string LogCollisionResult::getType () const
{
    return getTypeID ();
}

bool LogCollisionResult::ResultInfo::operator== (const ResultInfo& b) const
{
    if (frameA != b.frameA)
        return false;
    if (frameB != b.frameB)
        return false;
    if (geoNamesA != b.geoNamesA)
        return false;
    if (geoNamesB != b.geoNamesB)
        return false;

    if (result._aTb != b.result._aTb)
        return false;
    if (result._collisionPairs.size () != b.result._collisionPairs.size ())
        return false;
    for (std::size_t i = 0; i < result._collisionPairs.size (); i++) {
        if (result._collisionPairs[i].geoIdxA != b.result._collisionPairs[i].geoIdxA)
            return false;
        if (result._collisionPairs[i].geoIdxB != b.result._collisionPairs[i].geoIdxB)
            return false;
        if (result._collisionPairs[i].startIdx != b.result._collisionPairs[i].startIdx)
            return false;
        if (result._collisionPairs[i].size != b.result._collisionPairs[i].size)
            return false;
    }
    if (result._geomPrimIds != b.result._geomPrimIds)
        return false;
    if (result._nrBVTests != b.result._nrBVTests)
        return false;
    if (result._nrPrimTests != b.result._nrPrimTests)
        return false;

    return true;
}

bool LogCollisionResult::operator== (const SimulatorLog& b) const
{
    if (const LogCollisionResult* const entry = dynamic_cast< const LogCollisionResult* > (&b)) {
        if (*_positions != *entry->_positions)
            return false;
        // if (_results != entry->_results)
        //	return false;
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

std::list< SimulatorLogEntry::Ptr > LogCollisionResult::getLinkedEntries () const
{
    if (_positions == NULL)
        return std::list< SimulatorLogEntry::Ptr > ();
    else
        return std::list< SimulatorLogEntry::Ptr > (1, _positions);
}

bool LogCollisionResult::autoLink ()
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

SimulatorLogEntry::Ptr LogCollisionResult::createNew (SimulatorLogScope* parent) const
{
    return ownedPtr (new LogCollisionResult (parent));
}

std::string LogCollisionResult::getTypeID ()
{
    return "CollisionResult";
}

const std::vector< LogCollisionResult::ResultInfo >& LogCollisionResult::getResults () const
{
    return _results;
}

void LogCollisionResult::addResult (const CollisionStrategy::Result& result)
{
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

void LogCollisionResult::addResults (const std::vector< CollisionStrategy::Result >& results)
{
    for (const CollisionStrategy::Result& result : results) {
        addResult (result);
    }
}

LogPositions::Ptr LogCollisionResult::getPositions () const
{
    return _positions;
}
