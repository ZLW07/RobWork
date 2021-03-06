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

#include "ProximityModel.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/proximity/ProximityStrategy.hpp>

using namespace rw::proximity;

ProximityModel::~ProximityModel ()
{}

bool ProximityModel::addGeometry (rw::core::Ptr< rw::geometry::Geometry > geom, bool forceCopy)
{
    return owner->addGeometry (this, geom);
}

bool ProximityModel::addGeometry (const rw::geometry::Geometry& geom)
{
    return owner->addGeometry (this, geom);
}

bool ProximityModel::removeGeometry (const std::string& geom)
{
    return owner->removeGeometry (this, geom);
}

std::vector< std::string > ProximityModel::getGeometryIDs ()
{
    return owner->getGeometryIDs (this);
}

std::vector< rw::core::Ptr< rw::geometry::Geometry > > ProximityModel::getGeometries ()
{
    return owner->getGeometrys (this);
    ;
}
