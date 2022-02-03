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

#include <rw/core/Extension.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/Object.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>

#include <vector>

using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::core;
using namespace rw::models;

ProximityStrategy::ProximityStrategy () : _frameToModel (NULL, 100)
{}

ProximityStrategy::~ProximityStrategy ()
{}

ProximityModel::Ptr ProximityStrategy::getModel (const rw::kinematics::Frame* frame)
{
    ProximityModel::Ptr model = _frameToModel[*frame];
    return model;
}

bool ProximityStrategy::addModel (rw::models::Object::Ptr object)
{
    if (RigidObject::Ptr robj = object.cast< RigidObject > ()) {
        std::vector< Geometry::Ptr > geoms = robj->getGeometry ();
        for (Geometry::Ptr geom : geoms) {
            Frame* geoframe = geom->getFrame ();
            if (!hasModel (geoframe)) {
                _frameToModel[*geoframe] = createModel ();
                _frameToModel[*geoframe]->setFrame (geoframe);
            }
            ProximityModel::Ptr model = getModel (geoframe);
            model->addGeometry (geom);
        }
        return true;
    }
    return false;
}

bool ProximityStrategy::addModel (const Frame* frame, const rw::geometry::Geometry& geom)
{
    ProximityModel::Ptr model = getModel (frame);
    if (model == NULL) {
        model = createModel ();
    }
    model->setFrame (frame);
    bool res = addGeometry (model.get (), geom);
    if (res) {
        _frameToModel[*frame] = model;
    }
    return res;
}

bool ProximityStrategy::addModel (const Frame* frame, rw::geometry::Geometry::Ptr geom,
                                  bool forceCopy)
{
    ProximityModel::Ptr model = getModel (frame);
    if (model == NULL) {
        model = createModel ();
    }
    model->setFrame (frame);
    bool res = model->addGeometry (geom, forceCopy);
    if (res) {
        _frameToModel[*frame] = model;
    }
    return res;
}

bool ProximityStrategy::hasModel (const rw::kinematics::Frame* frame)
{
    if (!_frameToModel.has (*frame) || _frameToModel[*frame] == NULL) {
        // if (CollisionModelInfo::get(frame).size()>0)
        //    return true;
        return false;
    }
    return true;
}

void ProximityStrategy::clearFrame (const rw::kinematics::Frame* frame)
{
    if (!_frameToModel.has (*frame) || _frameToModel[*frame] == NULL)
        return;
    ProximityModel::Ptr model = _frameToModel[*frame];
    if (model == NULL)
        return;
    _frameToModel[*frame] = NULL;
    destroyModel (model.get ());
}

void ProximityStrategy::clearFrames ()
{
    _frameToModel.clear ();
}

ProximityStrategy::Factory::Factory () :
    ExtensionPoint< ProximityStrategy > ("rw.proximity.ProximityStrategy",
                                         "Extensions to create proximity strategies")
{}

std::vector< std::string > ProximityStrategy::Factory::getStrategies ()
{
    std::vector< std::string > ids;
    ProximityStrategy::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    ids.push_back ("RW");
    for (Extension::Descriptor& ext : exts) {
        ids.push_back (ext.getProperties ().get ("strategyID", ext.name));
    }
    return ids;
}

bool ProximityStrategy::Factory::hasStrategy (const std::string& strategy)
{
    std::string upper = strategy;
    std::transform (upper.begin (), upper.end (), upper.begin (), ::toupper);
    if (upper == "RW")
        return true;
    ProximityStrategy::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        std::string id = ext.getProperties ().get ("strategyID", ext.name);
        std::transform (id.begin (), id.end (), id.begin (), ::toupper);
        if (id == upper)
            return true;
    }
    return false;
}

ProximityStrategy::Ptr ProximityStrategy::Factory::makeStrategy (const std::string& strategy)
{
    std::string upper = strategy;
    std::transform (upper.begin (), upper.end (), upper.begin (), ::toupper);
    if (upper == "RW")
        return ownedPtr (new ProximityStrategyRW ());
    ProximityStrategy::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr& ext : exts) {
        std::string id = ext->getProperties ().get ("strategyID", ext->getName ());
        std::transform (id.begin (), id.end (), id.begin (), ::toupper);
        if (id == upper) {
            return ext->getObject ().cast< ProximityStrategy > ();
        }
    }
    return NULL;
}

std::vector< rw::core::Ptr< rw::geometry::Geometry > >
ProximityStrategy::getGeometrys (rw::proximity::ProximityModel* model)
{
    RW_THROW ("This Is a Virtual Function and needs to be replaced when Inherited");
    return std::vector< rw::core::Ptr< rw::geometry::Geometry > > ();
}
