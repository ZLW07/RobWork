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

#include "CollisionDetector.hpp"

#include "BasicFilterStrategy.hpp"
#include "CollisionStrategy.hpp"
#include "ProximityData.hpp"

#include <rw/common/ScopedTimer.hpp>
#include <rw/core/macros.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Object.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::core;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::geometry;

CollisionDetector::CollisionDetector (WorkCell::Ptr workcell) : ProximityCalculator (workcell, NULL)
{}

CollisionDetector::CollisionDetector (WorkCell::Ptr workcell, CollisionStrategy::Ptr strategy) :
    ProximityCalculator (workcell, strategy)
{}

CollisionDetector::CollisionDetector (WorkCell::Ptr workcell, CollisionStrategy::Ptr strategy,
                                      ProximityFilterStrategy::Ptr bpfilter) :
    ProximityCalculator (workcell, strategy)
{
    RW_ASSERT (bpfilter);
    setProximityFilterStrategy (bpfilter);
}

bool CollisionDetector::inCollision (const kinematics::State& state, ProximityData& proxdata) const
{
    ProximityStrategyData data;
    data.getCache () = proxdata._cache;

    proxdata._collisionData.collidingFrames.clear ();

    bool stopAtFirstContact = proxdata.getCollisionQueryType () == FirstContactFullInfo ||
                              proxdata.getCollisionQueryType () == FirstContactNoInfo;

    bool fullInfo = proxdata.getCollisionQueryType () == FirstContactFullInfo ||
                    proxdata.getCollisionQueryType () == AllContactsFullInfo;

    if (fullInfo) {
        data.setCollisionQueryType (CollisionStrategy::AllContacts);
    }
    else {
        data.setCollisionQueryType (CollisionStrategy::FirstContact);
    }
    if (stopAtFirstContact) {
        proxdata._collisionData._fullInfo.push_back (
            const_cast< CollisionDetector* > (this)->calculate (state, &data));
        if (proxdata._collisionData._fullInfo.front ().inCollision ()) {
            std::pair< Frame::Ptr, Frame::Ptr > colFrames =
                proxdata._collisionData._fullInfo.front ().getColidingFrames ();

            proxdata._collisionData.collidingFrames.insert (
                std::make_pair (colFrames.first.get (), colFrames.second.get ()));
        }
        else {
            proxdata._collisionData._fullInfo.clear ();
        }
    }
    else {
        ProximityCalculator::ResultType res =
            rw::core::ownedPtr (new std::vector< ProximityStrategyData > ());
        const_cast< CollisionDetector* > (this)->calculate (state, &data, res);

        for (ProximityStrategyData& d : *res) {
            if (d.inCollision ()) {
                proxdata._collisionData._fullInfo.push_back (d);
                std::pair< Frame::Ptr, Frame::Ptr > colFrames = d.getColidingFrames ();

                proxdata._collisionData.collidingFrames.insert (
                    std::make_pair (colFrames.first.get (), colFrames.second.get ()));
            }
        }
    }

    return proxdata._collisionData.collidingFrames.size () > 0;
}

bool CollisionDetector::inCollision (const State& state, QueryResult* result,
                                     bool stopAtFirstContact) const
{
    ProximityStrategyData data;
    data.setCollisionQueryType (CollisionStrategy::AllContacts);

    ProximityData res;
    if (result == NULL) {
        res.setCollisionQueryType (FirstContactFullInfo);
    }
    else {
        res.setCollisionQueryType (AllContactsFullInfo);

        if (stopAtFirstContact) {
            res.setCollisionQueryType (FirstContactFullInfo);
        }
    }

    bool coliding = inCollision (state, res);

    if (result != NULL) {
        *result = res._collisionData;
    }
    return coliding;
}

void CollisionDetector::addGeometry (rw::kinematics::Frame* frame,
                                     const rw::geometry::Geometry::Ptr geometry)
{
    ProximityCalculator::addGeometry (Frame::Ptr (frame), geometry);
}

void CollisionDetector::removeGeometry (rw::kinematics::Frame* frame,
                                        const rw::geometry::Geometry::Ptr geo)
{
    ProximityCalculator::removeGeometry (Frame::Ptr (frame), geo);
}

void CollisionDetector::removeGeometry (rw::kinematics::Frame* frame, const std::string geoid)
{
    ProximityCalculator::removeGeometry (Frame::Ptr (frame), geoid);
}

std::vector< std::string > CollisionDetector::getGeometryIDs (rw::kinematics::Frame* frame)
{
    return ProximityCalculator::getGeometryIDs (Frame::Ptr (frame));
}

bool CollisionDetector::hasGeometry (rw::kinematics::Frame* frame, const std::string& geometryId)
{
    return ProximityCalculator::hasGeometry (Frame::Ptr (frame), geometryId);
}

rw::geometry::Geometry::Ptr CollisionDetector::getGeometry (rw::kinematics::Frame* frame,
                                                            const std::string& geometryId)
{
    return ProximityCalculator::getGeometry (Frame::Ptr (frame), geometryId);
}
