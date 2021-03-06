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

#include "ProximityStrategyYaobi.hpp"

#include <rw/core/macros.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#include <vector>
#include <yaobi/yaobi.h>
#include <yaobi/yaobi_mesh_interface.h>
#include <yaobi/yaobi_tree_builder.h>

using namespace rw::common;
using namespace rw::core;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace {
Ptr< yaobi::CollModel > makeModelFromSoup (TriMesh::Ptr mesh, const double scale)
{
    unsigned char tri_stride (3);
    unsigned num_tris ((unsigned) mesh->getSize ());
    unsigned num_verts (num_tris * 3);
    yaobi::AppRealT* vertices = new yaobi::AppRealT[num_verts * 3];
    int* tris                 = new int[num_tris * 3];

    for (unsigned triIdx = 0, vertIdx = 0; triIdx < num_tris; triIdx++, vertIdx += 3) {
        const Triangle< double > face = mesh->getTriangle (triIdx);

        Vector3D< yaobi::Real > v0 = cast< yaobi::Real > (face[0] * scale);
        Vector3D< yaobi::Real > v1 = cast< yaobi::Real > (face[1] * scale);
        Vector3D< yaobi::Real > v2 = cast< yaobi::Real > (face[2] * scale);

        for (size_t j = 0; j < 3; j++) {
            vertices[vertIdx * 3 + 0 + j] = v0[j];
            vertices[vertIdx * 3 + 3 + j] = v1[j];
            vertices[vertIdx * 3 + 6 + j] = v2[j];
        }
        tris[vertIdx + 0] = vertIdx + 0;
        tris[vertIdx + 1] = vertIdx + 1;
        tris[vertIdx + 2] = vertIdx + 2;
    }

    yaobi::TriMeshInterface* tri = new yaobi::TriMeshInterface (
        num_verts, vertices, num_tris, tris, tri_stride, yaobi::OWN_DATA);

    Ptr< yaobi::CollModel > model = ownedPtr (new yaobi::CollModel (tri, yaobi::OWN_DATA));
    yaobi::build_obb_tree (*model, yaobi::OWN_DATA);

    // model->ShrinkToFit();
    return model;
}

// Convert Transform3D to Yaobi representation.
void toTransform (const Transform3D<>& tr, yaobi::Real T[3][4])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            T[i][j] = static_cast< yaobi::Real > (tr (i, j));
}

void collide (const yaobi::CollModel& ma, const Transform3D<>& wTa, const yaobi::CollModel& mb,
              const Transform3D<>& wTb, yaobi::CollideResult& result, yaobi::QueryType type)
{
    yaobi::Real ta[3][4];
    yaobi::Real tb[3][4];

    toTransform (wTa, ta);
    toTransform (wTb, tb);

    Collide (result, ta, ma, tb, mb, type);
}
}    // namespace

//----------------------------------------------------------------------
// ProximityStrategyYaobi

ProximityStrategyYaobi::ProximityStrategyYaobi ()
{}

rw::proximity::ProximityModel::Ptr ProximityStrategyYaobi::createModel ()
{
    YaobiProximityModel* model = new YaobiProximityModel (this);
    return ownedPtr (model);
}

void ProximityStrategyYaobi::destroyModel (rw::proximity::ProximityModel* model)
{
    RW_ASSERT (model != NULL);

    // model->models.clear();
    // YaobiProximityModel *pmodel = (YaobiProximityModel*) model.get();
}

bool ProximityStrategyYaobi::addGeometry (rw::proximity::ProximityModel* model,
                                          const rw::geometry::Geometry& geom)
{
    return addGeometry (model, ownedPtr (new Geometry (geom)), true);
}

bool ProximityStrategyYaobi::addGeometry (rw::proximity::ProximityModel* model,
                                          rw::geometry::Geometry::Ptr geom, bool forceCopy)
{
    RW_ASSERT (model != NULL);
    YaobiProximityModel* pmodel = (YaobiProximityModel*) model;

    for (RWYaobiModel& m : pmodel->models) {
        if (m.geo->getId () == geom->getId ()) {
            RW_THROW ("The specified geometry \"" + geom->getId () +
                      "\" (geometry identifiers are supposed to be unique) has "
                      "already been added to the proximity strategy model!");
            return false;
        }
    }

    YaobiModelPtr yaobimodel;
    GeometryData::Ptr gdata = geom->getGeometryData ();
    // first check if model is in cache
    if (_modelCache.has (geom->getId ())) {
        yaobimodel = _modelCache.get (geom->getId ());
    }
    else {
        TriMesh::Ptr mesh = gdata->getTriMesh (false);
        if (mesh->getSize () == 0)
            return false;

        yaobimodel = makeModelFromSoup (mesh, geom->getScale ());
    }
    pmodel->models.push_back (RWYaobiModel (geom, geom->getTransform (), yaobimodel));

    _allmodels.push_back (pmodel->models.back ());
    _geoIdToModelIdx[geom->getId ()].push_back ((int) _allmodels.size () - 1);
    return true;
}

bool ProximityStrategyYaobi::removeGeometry (rw::proximity::ProximityModel* model,
                                             const std::string& geomId)
{
    YaobiProximityModel* pmodel = (YaobiProximityModel*) model;
    int idx                     = -1;
    for (size_t i = 0; i < pmodel->models.size (); i++)
        if (pmodel->models[i].geo->getId () == geomId) {
            idx = (int) i;
            break;
        }
    if (idx < 0) {
        // RW_THROW("No geometry with id: \""<< geomId << "\" exist in proximity model!");
        return false;
    }

    _modelCache.remove (geomId);
    RWYaobiModelList::iterator iter = pmodel->models.begin ();
    for (; iter != pmodel->models.end (); ++iter) {
        if ((*iter).geo->getId () == geomId) {
            pmodel->models.erase (iter);
            return true;
        }
    }
    return false;
}

bool ProximityStrategyYaobi::doInCollision (ProximityModel::Ptr aModel, const Transform3D<>& wTa,
                                            ProximityModel::Ptr bModel, const Transform3D<>& wTb,
                                            ProximityStrategyData& data)
{
    bool firstContact      = data.getCollisionQueryType () == CollisionStrategy::FirstContact;
    bool isColliding       = false;
    YaobiProximityModel* a = (YaobiProximityModel*) aModel.get ();
    YaobiProximityModel* b = (YaobiProximityModel*) bModel.get ();

    CollisionResult& cres = data.getCollisionData ();
    cres.clear ();
    cres.a    = aModel;
    cres.b    = bModel;
    cres._aTb = inverse (wTa) * wTb;

    yaobi::QueryType qtype = yaobi::FIRST_CONTACT_ONLY;
    if (!firstContact)
        qtype = yaobi::ALL_CONTACTS;
    yaobi::CollideResult result;
    size_t nrOfCollidingGeoms = 0;
    int geoIdxA               = 0;
    int geoIdxB               = 0;
    for (const RWYaobiModel& ma : a->models) {
        for (const RWYaobiModel& mb : b->models) {
            //! Search for all contacting triangles
            collide (*ma.yaobimodel, wTa * ma.t3d, *mb.yaobimodel, wTb * mb.t3d, result, qtype);

            cres._nrBVTests += result.num_bv_tests;
            cres._nrPrimTests += result.num_tri_tests;

            if (result.IsColliding ()) {
                nrOfCollidingGeoms++;
                cres._collisionPairs.resize (nrOfCollidingGeoms);
                cres._collisionPairs.back ().geoIdxA = geoIdxA;
                cres._collisionPairs.back ().geoIdxB = geoIdxB;

                int startIdx = static_cast< int > (cres._geomPrimIds.size ());
                int size     = result.num_pairs;
                cres._collisionPairs.back ().startIdx = startIdx;
                cres._collisionPairs.back ().size     = size;

                cres._geomPrimIds.resize (startIdx + size);

                for (int j = 0; j < size; j++) {
                    cres._geomPrimIds[startIdx + j].first  = result.pairs[j].id1;
                    cres._geomPrimIds[startIdx + j].second = result.pairs[j].id2;
                }
            }

            if (firstContact && result.IsColliding ())
                return true;
            if (result.IsColliding ())
                isColliding = true;

            geoIdxB++;
        }
        geoIdxA++;
    }

    return isColliding;
}

void ProximityStrategyYaobi::clear ()
{
    // TODO: also clear cache
    //_frameModelMap.clear();
    _modelCache.clear ();
    _geoIdToModelIdx.clear ();
    _allmodels.clear ();

    clearFrames ();
}

void ProximityStrategyYaobi::getCollisionContacts (
    std::vector< CollisionStrategy::Contact >& contacts, rw::proximity::ProximityStrategyData& data)
{
    RW_THROW ("NOT IM PLEMENTED IN YAOBI COLLISION STRATEGY!");
}

std::vector< std::string >
ProximityStrategyYaobi::getGeometryIDs (rw::proximity::ProximityModel* model)
{
    std::vector< std::string > res;
    YaobiProximityModel* pmodel = (YaobiProximityModel*) model;
    for (RWYaobiModel& m : pmodel->models) {
        res.push_back (m.geo->getId ());
    }
    return res;
}
std::vector< rw::core::Ptr< rw::geometry::Geometry > >
ProximityStrategyYaobi::getGeometrys (rw::proximity::ProximityModel* model)
{
    std::vector< rw::core::Ptr< rw::geometry::Geometry > > res;
    YaobiProximityModel* pmodel = (YaobiProximityModel*) model;
    for (RWYaobiModel& m : pmodel->models) {
        res.push_back (m.geo);
    }
    return res;
}

CollisionStrategy::Ptr ProximityStrategyYaobi::make ()
{
    return ownedPtr (new ProximityStrategyYaobi);
}
