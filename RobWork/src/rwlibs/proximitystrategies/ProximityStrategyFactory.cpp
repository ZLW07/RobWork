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

#include "ProximityStrategyFactory.hpp"

#include <RobWorkConfig.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/core/macros.hpp>
#include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>

#ifdef RW_HAVE_PQP
#include "ProximityStrategyPQP.hpp"
#endif

#ifdef RW_HAVE_YAOBI
#include "ProximityStrategyYaobi.hpp"
#endif

#ifdef RW_HAVE_BULLET
#include "ProximityStrategyBullet.hpp"
#endif

#ifdef RW_HAVE_FCL
#include "ProximityStrategyFCL.hpp"
#endif

namespace {
const std::string RWStr ("RWPROX");
const std::string PQPStr ("PQP");
const std::string YAOBIStr ("YAOBI");
const std::string BulletStr ("BULLET");
const std::string FCLStr ("FCL");
}    // namespace

using namespace rwlibs::proximitystrategies;
using namespace rw::proximity;
using namespace rw;
using namespace rw::core;

rw::proximity::CollisionStrategy::Ptr ProximityStrategyFactory::makeDefaultCollisionStrategy ()
{
#ifdef RW_HAVE_PQP
    return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
#endif

#ifdef RW_HAVE_YAOBI
    return rw::core::ownedPtr (new ProximityStrategyYaobi ());
#endif

#ifdef RW_HAVE_BULLET
    return rw::core::ownedPtr (new ProximityStrategyBullet ());
#endif

#ifdef RW_HAVE_FCL
    return rw::core::ownedPtr (new ProximityStrategyFCL ());
#endif

    return rw::core::ownedPtr<> (new ProximityStrategyRW ());
}

rw::proximity::CollisionStrategy::Ptr
ProximityStrategyFactory::makeCollisionStrategy (const std::string& id)
{
    if (id == RWStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyRW ());
    }
#ifdef RW_HAVE_PQP
    if (id == PQPStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
    }
#endif

#ifdef RW_HAVE_YAOBI
    if (id == YAOBIStr) {
        return rw::core::ownedPtr (new ProximityStrategyYaobi ());
    }
#endif

#ifdef RW_HAVE_BULLET
    if (id == BulletStr) {
        return rw::core::ownedPtr (new ProximityStrategyBullet ());
    }
#endif

#ifdef RW_HAVE_FCL
    if (id == FCLStr) {
        return rw::core::ownedPtr (new ProximityStrategyFCL ());
    }
#endif

    RW_THROW ("No support for collision strategy with ID=" << StringUtil::quote (id));
    return NULL;
}

std::vector< std::string > ProximityStrategyFactory::getCollisionStrategyIDs ()
{
    std::vector< std::string > IDs;

#ifdef RW_HAVE_PQP
    IDs.push_back (PQPStr);
#endif

#ifdef RW_HAVE_YAOBI
    IDs.push_back (YAOBIStr);
#endif

#ifdef RW_HAVE_BULLET
    IDs.push_back (BulletStr);
#endif

#ifdef RW_HAVE_FCL
    IDs.push_back (FCLStr);
#endif

    IDs.push_back (RWStr);

    return IDs;
}

std::vector< std::string > ProximityStrategyFactory::getDistanceStrategyIDs ()
{
    std::vector< std::string > IDs;

#ifdef RW_HAVE_PQP
    IDs.push_back (PQPStr);
#endif

#ifdef RW_HAVE_FCL
    IDs.push_back (FCLStr);
#endif

    return IDs;
}

rw::proximity::DistanceStrategy::Ptr ProximityStrategyFactory::makeDefaultDistanceStrategy ()
{
#ifdef RW_HAVE_PQP
    return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
#endif

#ifdef RW_HAVE_FCL
    return rw::core::ownedPtr (new ProximityStrategyFCL ());
#endif

    RW_THROW ("No default distance strategies available");
    return NULL;
}

rw::proximity::DistanceStrategy::Ptr
ProximityStrategyFactory::makeDistanceStrategy (const std::string& id)
{
#ifdef RW_HAVE_PQP
    if (id == PQPStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
    }
#endif
#ifdef RW_HAVE_FCL
    if (id == FCLStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyFCL ());
    }
#endif

    RW_THROW ("No support for distance strategy with ID=" << StringUtil::quote (id));
    return NULL;
}

std::vector< std::string > ProximityStrategyFactory::getDistanceMultiStrategyIDs ()
{
   std::vector< std::string > IDs;

#ifdef RW_HAVE_PQP
    IDs.push_back (PQPStr);
#endif

#ifdef RW_HAVE_FCL
    IDs.push_back (FCLStr);
    std::cout << "FCL strategy" << std::endl;
#endif
    std::cout << "ID's fetched" << std::endl;
    return IDs;
}

rw::proximity::DistanceMultiStrategy::Ptr
ProximityStrategyFactory::makeDefaultDistanceMultiStrategy ()
{
#ifdef RW_HAVE_PQP
    return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
#endif
#ifdef RW_HAVE_FCL
    return rw::core::ownedPtr (new ProximityStrategyFCL ());
#endif
    RW_THROW ("No default distance multi strategies available");
    return NULL;
}

rw::proximity::DistanceMultiStrategy::Ptr
ProximityStrategyFactory::makeDistanceMultiStrategy (const std::string& id)
{
#ifdef RW_HAVE_PQP
    if (id == PQPStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyPQP ());
    }
#endif
#ifdef RW_HAVE_FCL
    if (id == FCLStr) {
        return rw::core::ownedPtr<> (new ProximityStrategyFCL ());
    }
#endif

    RW_THROW ("No support for distance multi strategy with ID=" << StringUtil::quote (id));
    return NULL;
}
