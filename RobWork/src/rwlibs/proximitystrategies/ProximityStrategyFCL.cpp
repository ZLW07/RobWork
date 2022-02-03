/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#if __GNUC__ < 5 || (__GNUC__ == 5 && __GNUC_MINOR__ <= 4)
#define BOOST_MATH_DISABLE_FLOAT128
#endif

#include "ProximityStrategyFCL.hpp"

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#if FCL_VERSION_LESS_THEN_0_6_0
#include <fcl/BVH/BVH_internal.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/intersect.h>
#else
#include <fcl/common/types.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/triangle_distance.h>
#include <fcl/narrowphase/distance.h>
#endif

#if FCL_VERSION_LESS_THEN_0_6_0
using fclTriangleDistance = fcl::TriangleDistance;
#else
using fclTriangleDistance = fcl::detail::TriangleDistance< double >;
#endif

using rw::core::ownedPtr;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rwlibs::proximitystrategies;

namespace {
#ifdef FCL_VERSION_LESS_THEN_0_6_0

fcl::Matrix3f toFCL (const Rotation3D<>& rwR)
{
    return fcl::Matrix3f (rwR (0, 0),
                          rwR (0, 1),
                          rwR (0, 2),
                          rwR (1, 0),
                          rwR (1, 1),
                          rwR (1, 2),
                          rwR (2, 0),
                          rwR (2, 1),
                          rwR (2, 2));
}
fcl::Vec3f toFCL (const Vector3D<>& rwV)
{
    return fcl::Vec3f (rwV[0], rwV[1], rwV[2]);
}
fcl::Transform3f toFCL (const Transform3D<>& rwT)
{
    fcl::Matrix3f rotation (rwT (0, 0),
                            rwT (0, 1),
                            rwT (0, 2),
                            rwT (1, 0),
                            rwT (1, 1),
                            rwT (1, 2),
                            rwT (2, 0),
                            rwT (2, 1),
                            rwT (2, 2));
    fcl::Vec3f translation (rwT (0, 3), rwT (1, 3), rwT (2, 3));
    fcl::Transform3f fclT (rotation, translation);

    return fclT;
}

Vector3D< double > fromFCL (const fcl::Vec3f& rwV)
{
    return Vector3D< double > (rwV[0], rwV[1], rwV[2]);
}

Rotation3D<> fromFCL (const fcl::Matrix3f& rwR)
{
    return Rotation3D<> (rwR (0, 0),
                         rwR (0, 1),
                         rwR (0, 2),
                         rwR (1, 0),
                         rwR (1, 1),
                         rwR (1, 2),
                         rwR (2, 0),
                         rwR (2, 1),
                         rwR (2, 2));
}

using rw_AABB   = fcl::AABB;
using rw_OBB    = fcl::OBB;
using rw_RSS    = fcl::RSS;
using rw_OBBRSS = fcl::OBBRSS;
using rw_kIOS   = fcl::kIOS;
using rw_KDOP16 = fcl::KDOP< 16 >;
using rw_KDOP18 = fcl::KDOP< 18 >;
using rw_KDOP24 = fcl::KDOP< 24 >;
using fclVec3   = fcl::Vec3f;

using fclContact = fcl::Contact;
#else
fcl::Matrix3< double >& toFCL (Rotation3D<>& rwR)
{
    return rwR.e ();
}

fcl::Vector3< double >& toFCL (Vector3D<>& rwV)
{
    return rwV.e ();
}

fcl::Transform3< double > toFCL (const Transform3D<>& rwT)
{
    return fcl::Transform3< double > (rwT.e ());
}

Vector3D<double> fromFCL (fcl::Vector3< double >& rwV)
{
    return Vector3D<double>(rwV);
}

fcl::Matrix3< double >& fromFCL (fcl::Matrix3< double >& rwV)
{
    return rwV;
}

using rw_AABB    = fcl::AABB< double >;
using rw_OBB     = fcl::OBB< double >;
using rw_RSS     = fcl::RSS< double >;
using rw_OBBRSS  = fcl::OBBRSS< double >;
using rw_kIOS    = fcl::kIOS< double >;
using rw_KDOP16  = fcl::KDOP< double, 16 >;
using rw_KDOP18  = fcl::KDOP< double, 18 >;
using rw_KDOP24  = fcl::KDOP< double, 24 >;
using fclContact = fcl::Contact< double >;
using fclVec3    = fcl::Vector3< double >;
#endif
}    // namespace

ProximityStrategyFCL::ProximityStrategyFCL (BV bv) :
    _bv (bv), _fclCollisionRequest (new fclCollisionRequest ()),
    _fclDistanceRequest (new fclDistanceRequest ()), _fclDistanceResult (new fclDistanceResult ())
{
    // Setup defaults
    _fclDistanceRequest->enable_nearest_points = true;
    _fclCollisionRequest->enable_contact       = true;
    _collectFCLResults                         = false;
}

ProximityStrategyFCL::~ProximityStrategyFCL ()
{
    delete _fclCollisionRequest;
    delete _fclDistanceRequest;
    delete _fclDistanceResult;
}

ProximityModel::Ptr ProximityStrategyFCL::createModel ()
{
    ProximityModel::Ptr p = ownedPtr (new FCLProximityModel (this));
    return p;
}

void ProximityStrategyFCL::destroyModel (ProximityModel* model)
{
    /* The created model was placed in a smart pointer
     * thus the memory deallocation is happening
     * automatically.
     */
}

bool ProximityStrategyFCL::addGeometry (ProximityModel* model, const Geometry& geom)
{
    switch (_bv) {
        case BV::AABB: return addGeometry< rw_AABB > (model, geom); break;
        case BV::OBB: return addGeometry< rw_OBB > (model, geom); break;
        case BV::RSS: return addGeometry< rw_RSS > (model, geom); break;
        case BV::OBBRSS: return addGeometry< rw_OBBRSS > (model, geom); break;
        case BV::kIOS: return addGeometry< rw_kIOS > (model, geom); break;
        case BV::KDOP16: return addGeometry< rw_KDOP16 > (model, geom); break;
        case BV::KDOP18: return addGeometry< rw_KDOP18 > (model, geom); break;
        case BV::KDOP24: return addGeometry< rw_KDOP24 > (model, geom); break;
        default:
            RW_THROW ("Implementation error! Support for the chosen FCL bounding volume has not "
                      "been properly implemented!");
            break;
    }

    return false;
}

bool ProximityStrategyFCL::addGeometry (rw::proximity::ProximityModel* model,
                                        rw::geometry::Geometry::Ptr geom, bool forceCopy)
{
    switch (_bv) {
        case BV::AABB: return addGeometry< rw_AABB > (model, geom, forceCopy); break;
        case BV::OBB: return addGeometry< rw_OBB > (model, geom, forceCopy); break;
        case BV::RSS: return addGeometry< rw_RSS > (model, geom, forceCopy); break;
        case BV::OBBRSS: return addGeometry< rw_OBBRSS > (model, geom, forceCopy); break;
        case BV::kIOS: return addGeometry< rw_kIOS > (model, geom, forceCopy); break;
        case BV::KDOP16: return addGeometry< rw_KDOP16 > (model, geom, forceCopy); break;
        case BV::KDOP18: return addGeometry< rw_KDOP18 > (model, geom, forceCopy); break;
        case BV::KDOP24: return addGeometry< rw_KDOP24 > (model, geom, forceCopy); break;
        default:
            RW_THROW ("Implementation error! Support for the chosen FCL bounding volume has not "
                      "been properly implemented!");
            break;
    }

    return false;
}

template< typename BV_t >
bool ProximityStrategyFCL::addGeometry (ProximityModel* model, const Geometry& geom)
{
    return addGeometry (model, ownedPtr (new Geometry (geom)), true);
}

template< typename BV_t >
bool ProximityStrategyFCL::addGeometry (rw::proximity::ProximityModel* model,
                                        rw::geometry::Geometry::Ptr geom, bool forceCopy)
{
    // The data is always copied in the called addGeometry function

    RW_ASSERT (model != 0);
    FCLProximityModel* pmodel = dynamic_cast< FCLProximityModel* > (model);

    /*std::cout << "geo: " << geom->getId () << std::endl;
    auto tri = geom->getGeometryData()->getTriMesh();
    for (size_t i = 0; i < tri->getSize(); i ++ ){
        std::cout << "Triangle:" << tri->getTriangle(i) << std::endl;
    }*/

    /* Verify that the geometry has not already been added
     * - Could also implement a cache, like for PQP, and accept adding the same geometry by just
     * fetching it from the cache. */
    for (const auto& m : pmodel->models) {
        if (m.geo->getId () == geom->getId ()) {
            RW_THROW ("The specified geometry \"" + geom->getId () +
                      "\" (geometry identifiers are supposed to be unique) has "
                      "already been added to the FCL proximity strategy model!");
            return false;
        }
    }

    const double scale            = geom->getScale ();
    TriMesh::Ptr mesh             = geom->getGeometryData ()->getTriMesh (false);
    std::size_t numberOfTriangles = mesh->getSize ();
    /* There are 3 vertices in a triangle, thus a factor of 3 is used to specify the amount of data
     * to allocate */
    std::size_t numberOfVertices = numberOfTriangles * 3;

    rw::core::Ptr< fcl::BVHModel< BV_t > > fclBVHModel = ownedPtr (new fcl::BVHModel< BV_t >);

    int returnCode = 0;
    returnCode     = fclBVHModel->beginModel (numberOfTriangles, numberOfVertices);
    if (returnCode != fcl::BVH_OK) {
        /* error - do diagnosis on the error codes and throw a proper exception? */
        return false;
    }

    for (std::size_t triIndex = 0; triIndex < numberOfTriangles; ++triIndex) {
        const Triangle<> face  = mesh->getTriangle (triIndex);
        const Vector3D<> v0Tmp = face[0] * scale;
        const Vector3D<> v1Tmp = face[1] * scale;
        const Vector3D<> v2Tmp = face[2] * scale;

#ifdef FCL_VERSION_LESS_THEN_0_6_0
        fcl::Vec3f v0, v1, v2;
        for (std::size_t i = 0; i < 3; ++i) {
            v0[i] = v0Tmp[i];
            v1[i] = v1Tmp[i];
            v2[i] = v2Tmp[i];
        }
#else
        fcl::Vector3d v0 = v0Tmp.e ();
        fcl::Vector3d v1 = v1Tmp.e ();
        fcl::Vector3d v2 = v2Tmp.e ();
#endif
        /* mband todo:
         * - How to ensure that double is the type supported and that it can just be copied even
         * when SSE is enabled in the FCL library, thus causing it to be floats (or some other
         * aligned/optimised type layout)?
         */
        returnCode = fclBVHModel->addTriangle (v0, v1, v2);
        if (returnCode != fcl::BVH_OK) {
            /* error - do diagnosis on the error codes and throw a proper exception? */
            return false;
        }
    }

    returnCode = fclBVHModel->endModel ();
    if (returnCode != fcl::BVH_OK) {
        /* error - do diagnosis on the error codes and throw a proper exception? */
        return false;
    }

    /* Could use .scast to use static_cast instead of dynamic_cast and avoid the overhead caused by
     * runtime verification that the converted object is complete. */
    FCLBVHModelPtr fclBVHModelPtr = fclBVHModel.template cast< fclCollisionGeometry > ();
    if (!fclBVHModelPtr) {
        return false;
    }
    FCLModel fclModel (geom, geom->getTransform (), fclBVHModelPtr);
    pmodel->models.push_back (fclModel);
    return true;
}

bool ProximityStrategyFCL::removeGeometry (rw::proximity::ProximityModel* model,
                                           const std::string& geomId)
{
    RW_ASSERT (model != 0);
    FCLProximityModel* pmodel = dynamic_cast< FCLProximityModel* > (model);

    for (auto it = pmodel->models.begin (); it != pmodel->models.end (); ++it) {
        if ((*it).geo->getId () == geomId) {
            pmodel->models.erase (it);
            return true;
        }
    }

    /* Nothing was erased */
    return false;
}

std::vector< std::string >
ProximityStrategyFCL::getGeometryIDs (rw::proximity::ProximityModel* model)
{
    RW_ASSERT (model != 0);
    FCLProximityModel* pmodel = dynamic_cast< FCLProximityModel* > (model);

    std::vector< std::string > geometryIDs;
    geometryIDs.reserve (pmodel->models.size ());
    for (const auto& m : pmodel->models) {
        geometryIDs.push_back (m.geo->getId ());
    }

    return geometryIDs;
}

std::vector< rw::core::Ptr< rw::geometry::Geometry > >
ProximityStrategyFCL::getGeometrys (rw::proximity::ProximityModel* model)
{
    RW_ASSERT (model != 0);
    FCLProximityModel* pmodel = dynamic_cast< FCLProximityModel* > (model);

    std::vector< rw::core::Ptr< rw::geometry::Geometry > > geometrys;
    geometrys.reserve (pmodel->models.size ());
    for (const auto& m : pmodel->models) {
        geometrys.push_back (m.geo);
    }

    return geometrys;
}

void ProximityStrategyFCL::clear ()
{
    /* Assumes that all the ProximityModels are attached/connected to frames and thus being cleared
     */
    clearFrames ();
}

bool ProximityStrategyFCL::doInCollision (rw::proximity::ProximityModel::Ptr a,
                                          const rw::math::Transform3D<>& wTa,
                                          rw::proximity::ProximityModel::Ptr b,
                                          const rw::math::Transform3D<>& wTb,
                                          rw::proximity::ProximityStrategyData& data)
{
    RW_ASSERT (a != nullptr);
    RW_ASSERT (b != nullptr);

    bool inCollision = false;

    FCLProximityModel* aModel = dynamic_cast< FCLProximityModel* > (a.get ());
    FCLProximityModel* bModel = dynamic_cast< FCLProximityModel* > (b.get ());

    _fclCollisionResults.clear ();

    rw::proximity::CollisionResult& collisionResult = data.getCollisionData ();
    collisionResult.clear ();

    bool firstContact = false;
    switch (data.getCollisionQueryType ()) {
        case CollisionStrategy::FirstContact:
            firstContact                           = true;
            _fclCollisionRequest->num_max_contacts = 1;
            break;
        case CollisionStrategy::AllContacts:
            _fclCollisionRequest->num_max_contacts = std::numeric_limits< size_t >::max ();
            break;
        default:
            RW_THROW ("There is no implementation for the chosen CollisionStrategy type '"
                      << data.getCollisionQueryType () << "'!");
            break;
    }

    /* TODO: Could use the fcl::CollisionResult that is created in _fclCollisionResults, and avoid
     * having the cost of copying data into that collection */
    fclCollisionResult fclCollisionResult;

    size_t geoIdxA = 0;
    size_t geoIdxB = 0;
    for (const auto& ma : aModel->models) {
        for (const auto& mb : bModel->models) {
            fclCollisionResult.clear ();
            fcl::collide (ma.model.get (),
                          toFCL (wTa * ma.t3d),
                          mb.model.get (),
                          toFCL (wTb * mb.t3d),
                          *_fclCollisionRequest,
                          fclCollisionResult);

            if (fclCollisionResult.isCollision ()) {
                inCollision = true;

                collisionResult.a    = a;
                collisionResult.b    = b;
                collisionResult._aTb = inverse (wTa) * wTb;

                rw::proximity::CollisionResult::CollisionPair collisionPair;
                collisionPair.geoIdxA  = static_cast< int > (geoIdxA);
                collisionPair.geoIdxB  = static_cast< int > (geoIdxB);
                collisionPair.startIdx = static_cast< int > (collisionResult._geomPrimIds.size ());
                collisionPair.size     = fclCollisionResult.numContacts ();

                collisionResult._collisionPairs.push_back (collisionPair);

                for (size_t j = 0; j < fclCollisionResult.numContacts (); ++j) {
                    const fclContact& contact = fclCollisionResult.getContact (j);
                    collisionResult._geomPrimIds.push_back (
                        std::make_pair (contact.b1, contact.b2));
                }

                if (_collectFCLResults) {
                    _fclCollisionResults.push_back (fclCollisionResult);
                }

                if (firstContact) {
                    return inCollision;
                }
            }
            ++geoIdxB;
        }
        ++geoIdxA;
    }

    return inCollision;
}

void ProximityStrategyFCL::getCollisionContacts (
    std::vector< CollisionStrategy::Contact >& contacts, rw::proximity::ProximityStrategyData& data)
{
    RW_THROW ("The getCollisionContacts function is not implemented (yet)!");
}

rw::proximity::DistanceStrategy::Result& ProximityStrategyFCL::doDistance (
    rw::proximity::ProximityModel::Ptr a, const rw::math::Transform3D<>& wTa,
    rw::proximity::ProximityModel::Ptr b, const rw::math::Transform3D<>& wTb,
    class rw::proximity::ProximityStrategyData& data)
{
    RW_ASSERT (a != nullptr);
    RW_ASSERT (b != nullptr);

    FCLProximityModel* aModel = dynamic_cast< FCLProximityModel* > (a.get ());
    FCLProximityModel* bModel = dynamic_cast< FCLProximityModel* > (b.get ());

    _fclDistanceResult->clear ();

    rw::proximity::DistanceStrategy::Result& res = data.getDistanceData ();
    res.clear ();

    res.a = a;
    res.b = b;

    fclDistanceResult AfclDistanceResult;

    _fclDistanceRequest->abs_err = data.abs_err;
    _fclDistanceRequest->rel_err = data.rel_err;

    res.distance = std::numeric_limits< double >::max ();

    size_t geoIdxA = 0;
    size_t geoIdxB = 0;
    for (const auto& ma : aModel->models) {
        geoIdxB = 0;
        for (const auto& mb : bModel->models) {
            double minDistance = 0;
            /* mband TODO:
             * For some FCL Bounding Volumes (.e.g BV_OBB), when used for both collision objects,
             * there is no valid distance calculator (i.e. the following is output in cerr
             * (hardcoded from the fcl library)): "Warning: distance function between node type 2
             * and node type 2 is not supported"
             *
             *  ^---> How to properly report this issue/problem? Throw a patch upstream to make
             * their error reporting/handling more library like/tolerant (e.g. not writing to
             * std::cerr and giving some sort of error). Or make a patch that fixes it locally for
             * the FCL that is distributed together with RobWork?
             */
            minDistance = fcl::distance (ma.model.get (),
                                         toFCL (wTa * ma.t3d),
                                         mb.model.get (),
                                         toFCL (wTb * mb.t3d),
                                         *_fclDistanceRequest,
                                         AfclDistanceResult);
            RW_ASSERT (fabs (minDistance - AfclDistanceResult.min_distance) < 1.0e-16);

            // Only update data if a shorter distance has been found
            if (minDistance < res.distance) {
                res.distance = minDistance;

                res.geoIdxA = static_cast< int > (geoIdxA);
                res.geoIdxB = static_cast< int > (geoIdxB);
                res.idx1    = AfclDistanceResult.b1;
                res.idx2    = AfclDistanceResult.b2;

                if (_fclDistanceRequest->enable_nearest_points) {
                    for (size_t i = 0; i < 3; ++i) {
                        res.p1[i] = AfclDistanceResult.nearest_points[0][i];
                        res.p2[i] = AfclDistanceResult.nearest_points[1][i];
                    }
                }

                if (_collectFCLResults) {
                    *_fclDistanceResult = AfclDistanceResult;
                }
            }
            if (minDistance <= 0) {
                RW_LOG_DEBUG ("The objects are in collision! But still calculating a distance.");
            }
            ++geoIdxB;
        }
        ++geoIdxA;
    }

    return res;
}

fclCollisionRequest& ProximityStrategyFCL::getCollisionRequest ()
{
    return *_fclCollisionRequest;
}

fclDistanceRequest& ProximityStrategyFCL::getDistanceRequest ()
{
    return *_fclDistanceRequest;
}

fclCollisionResult& ProximityStrategyFCL::getCollisionResult (std::size_t index)
{
    /* Using .at(...) will throw an exception if index is out of range */
    return _fclCollisionResults.at (index);
}

fclDistanceResult& ProximityStrategyFCL::getDistanceResult ()
{
    return *_fclDistanceResult;
}
namespace {
rw::geometry::Triangle< double > getTriangleFromModel (ProximityStrategyFCL::FCLBVHModelPtr model,
                                                       size_t index, ProximityStrategyFCL::BV bv)
{
    using Triangle = rw::geometry::Triangle< double >;
    using BV       = ProximityStrategyFCL::BV;

    Triangle tri;
    if (bv == BV::AABB) {
        auto bvh              = model.cast< fcl::BVHModel< rw_AABB > > ();
        fcl::Triangle fcl_tri = bvh->tri_indices[index];
        tri                   = Triangle (fromFCL (bvh->vertices[fcl_tri[0]]),
                        fromFCL (bvh->vertices[fcl_tri[1]]),
                        fromFCL (bvh->vertices[fcl_tri[2]]));
    }
    else if (bv == BV::OBB) {
        auto bvh              = model.cast< fcl::BVHModel< rw_OBB > > ();
        fcl::Triangle fcl_tri = bvh->tri_indices[index];
        tri                   = Triangle (fromFCL (bvh->vertices[fcl_tri[0]]),
                        fromFCL (bvh->vertices[fcl_tri[1]]),
                        fromFCL (bvh->vertices[fcl_tri[2]]));
    }
    else if (bv == BV::RSS) {
        auto bvh              = model.cast< fcl::BVHModel< rw_RSS > > ();
        fcl::Triangle fcl_tri = bvh->tri_indices[index];
        tri                   = Triangle  (fromFCL (bvh->vertices[fcl_tri[0]]),
                        fromFCL (bvh->vertices[fcl_tri[1]]),
                        fromFCL (bvh->vertices[fcl_tri[2]]));
    }
    else if (bv == BV::OBBRSS) {
        auto bvh              = model.cast< fcl::BVHModel< rw_OBBRSS > > ();
        fcl::Triangle fcl_tri = bvh->tri_indices[index];
        tri                   = Triangle (fromFCL (bvh->vertices[fcl_tri[0]]),
                        fromFCL (bvh->vertices[fcl_tri[1]]),
                        fromFCL (bvh->vertices[fcl_tri[2]]));
    }
    else if (bv == BV::kIOS) {
        auto bvh              = model.cast< fcl::BVHModel< rw_kIOS > > ();
        fcl::Triangle fcl_tri = bvh->tri_indices[index];
        tri                   = Triangle (fromFCL (bvh->vertices[fcl_tri[0]]),
                        fromFCL (bvh->vertices[fcl_tri[1]]),
                        fromFCL (bvh->vertices[fcl_tri[2]]));
    }
    else {
        RW_THROW ("Implementation error! Support for the chosen FCL bounding volume has not "
                  "been properly implemented!");
    }
    return tri;
}
}    // namespace
std::pair< rw::math::Vector3D<>, rw::math::Vector3D<> >
ProximityStrategyFCL::getSurfaceNormals (rw::proximity::DistanceMultiStrategy::Result& res, int idx)
{
    // get tris from FCL_models and compute triangle normal

    FCLProximityModel* a = (FCLProximityModel*) res.a.get ();
    FCLProximityModel* b = (FCLProximityModel*) res.b.get ();

    if (a->getGeometryIDs ().size () > 1 || b->getGeometryIDs ().size () > 1) {
        RW_THROW (" multiple geoms on one frame is not supported for normal extraction yet!");
    }

    int p1id = res.p1prims[idx];
    int p2id = res.p2prims[idx];

    rw::geometry::Triangle<double> atri = getTriangleFromModel(a->models[0].model,p1id,this->_bv);
    rw::geometry::Triangle<double> btri = getTriangleFromModel(b->models[0].model,p2id,this->_bv);

    rw::math::Vector3D<>& atri_p1 = atri[0];
    rw::math::Vector3D<>& atri_p2 = atri[1];
    rw::math::Vector3D<>& atri_p3 = atri[2];

    rw::math::Vector3D<>& btri_p1 = btri[0];
    rw::math::Vector3D<>& btri_p2 = btri[1];
    rw::math::Vector3D<>& btri_p3 = btri[2];

    rw::math::Vector3D<> n_p1 =
        a->models[0].t3d.R () * cross (atri_p2 - atri_p1, atri_p3 - atri_p1);
    rw::math::Vector3D<> n_p2 =
        b->models[0].t3d.R () * cross (btri_p2 - btri_p1, btri_p3 - btri_p1);

    return std::make_pair (n_p1, n_p2);
}

//######################################################################
//####################### multi distance implementation ################
//######################################################################
#include <rw/common/Timer.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::math;
class FCL_MultiDistanceResult
{
  public:
    FCL_MultiDistanceResult () : num_bv_tests (0), num_tri_tests (0) { clear (); };

    ~FCL_MultiDistanceResult (){};

    void clear ()
    {
        p1s.clear ();
        p2s.clear ();
        id1s.clear ();
        id2s.clear ();
        distances.clear ();
    }
    // stats

    int num_bv_tests;
    int num_tri_tests;
    double query_time_secs;

    // Transforform from model 1 to model 2
    rw::math::Transform3D< double > T;

    double rel_err;
    double abs_err;

    double distance;
    rw::math::Vector3D< double > p1;
    rw::math::Vector3D< double > p2;

    /*
     * LPE: Variables for storing the closest points of all triangles less than a certain distance
     * apart
     */
    std::vector< rw::math::Vector3D< double > > p1s;
    std::vector< rw::math::Vector3D< double > > p2s;
    std::vector< int > id1s;
    std::vector< int > id2s;

    std::vector< double > distances;

    int qsize;

    // statistics

    int NumBVTests () { return num_bv_tests; }
    int NumTriTests () { return num_tri_tests; }
    double QueryTimeSecs () { return query_time_secs; }

    // The following distance and points established the minimum distance
    // for the models, within the relative and absolute error bounds
    // specified.

    double Distance () { return distance; }
    rw::math::Vector3D< double >& P1 () { return p1; }
    rw::math::Vector3D< double >& P2 () { return p2; }
};

template< class BV_t >
void DistanceMultiThresholdRecurse (FCL_MultiDistanceResult& res,
                                    Transform3D< double > T,    // b2 relative to b1
                                    rw::core::Ptr< fcl::BVHModel< BV_t > >& o1, int b1,
                                    rw::core::Ptr< fcl::BVHModel< BV_t > >& o2, int b2)
{
    double sz1 = o1->getBV (b1).bv.size ();    // get size of object
    double sz2 = o2->getBV (b2).bv.size ();    // get size of object

    int l1 = o1->getBV (b1).isLeaf ();
    int l2 = o2->getBV (b2).isLeaf ();

    // If both models are a single triangle
    if (l1 && l2) {
        // both leaves.  Test the triangles beneath them.

        res.num_tri_tests++;

        fclVec3 p, q;

        int index1 = -o1->getBV (b1).first_child - 1;
        int index2 = -o2->getBV (b2).first_child - 1;

        fcl::Triangle t1 = o1->tri_indices[index1];
        fcl::Triangle t2 = o2->tri_indices[index2];

        fclVec3 T1[3] = {o1->vertices[t1[0]], o1->vertices[t1[1]], o1->vertices[t1[2]]};
        fclVec3 T2[3] = {o2->vertices[t2[0]], o2->vertices[t2[1]], o2->vertices[t2[2]]};

        double d =
            fclTriangleDistance::triDistance (T1, T2, toFCL (res.T.R ()), toFCL (res.T.P ()), p, q);
        if (d < res.distance) {
            res.distances.push_back (d);
            res.p1s.push_back (Vector3D< double > (p[0], p[1], p[2]));
            res.p2s.push_back (Vector3D< double > (q[0], q[1], q[2]));
            res.id1s.push_back (index1);
            res.id2s.push_back (index2);

            // into c.s. 2 later
            // o1->last_tri = t1;
            // o2->last_tri = t2;
        }
        return;
    }

    // First, perform distance tests on the children. Then traverse
    // them recursively, but test the closer pair first, the further
    // pair second.

    int a1, a2, c1, c2;    // new bv tests 'a' and 'c'

    if (l2 || (!l1 && (sz1 > sz2))) {
        // visit the children of b1

        a1 = o1->getBV (b1).leftChild ();
        a2 = b2;
        c1 = o1->getBV (b1).rightChild ();
        c2 = b2;
    }
    else {
        // visit the children of b2

        a1 = b1;
        a2 = o2->getBV (b2).leftChild ();
        c1 = b1;
        c2 = o2->getBV (b2).rightChild ();
    }

    res.num_bv_tests += 2;

    double d1 =
        fcl::distance (toFCL (T.R ()), toFCL (T.P ()), o1->getBV (a1).bv, o2->getBV (a2).bv);
    double d2 =
        fcl::distance (toFCL (T.R ()), toFCL (T.P ()), o1->getBV (c1).bv, o2->getBV (c2).bv);

    if (d2 < d1) {
        if ((d2 < (res.distance - res.abs_err)) || (d2 * (1 + res.rel_err) < res.distance)) {
            DistanceMultiThresholdRecurse (res, T, o1, c1, o2, c2);
        }

        if ((d1 < (res.distance - res.abs_err)) || (d1 * (1 + res.rel_err) < res.distance)) {
            DistanceMultiThresholdRecurse (res, T, o1, a1, o2, a2);
        }
    }
    else {
        if ((d1 < (res.distance - res.abs_err)) || (d1 * (1 + res.rel_err) < res.distance)) {
            DistanceMultiThresholdRecurse (res, T, o1, a1, o2, a2);
        }

        if ((d2 < (res.distance - res.abs_err)) || (d2 * (1 + res.rel_err) < res.distance)) {
            DistanceMultiThresholdRecurse (res, T, o1, c1, o2, c2);
        }
    }
    return;
}

template< class BV_t >
int FCL_DistanceMultiThreshold (FCL_MultiDistanceResult& res, double threshold,
                                rw::math::Transform3D< double > T1,
                                rw::core::Ptr< fclCollisionGeometry >& o1_cg,
                                rw::math::Transform3D< double > T2,
                                rw::core::Ptr< fclCollisionGeometry >& o2_cg, double rel_err,
                                double abs_err)
{
    rw::core::Ptr< fcl::BVHModel< BV_t > > o1 = o1_cg.cast< fcl::BVHModel< BV_t > > ();
    rw::core::Ptr< fcl::BVHModel< BV_t > > o2 = o2_cg.cast< fcl::BVHModel< BV_t > > ();

    rw::common::Timer timer;
    timer.reset ();

    // make sure that the models are built
    if (o1->build_state != fcl::BVHBuildState::BVH_BUILD_STATE_PROCESSED)
        return fcl::BVHReturnCode::BVH_ERR_BUILD_EMPTY_MODEL;
    if (o2->build_state != fcl::BVHBuildState::BVH_BUILD_STATE_PROCESSED)
        return fcl::BVHReturnCode::BVH_ERR_BUILD_EMPTY_MODEL;

    // Okay, compute what transform [R,T] that takes us from cs2 to cs1.
    res.T = rw::math::inverse (T1) * T2;

    res.distance = threshold;
    res.p1       = {0, 0, 0};
    res.p2       = {0, 0, 0};

    // initialize error bounds

    res.abs_err = abs_err;
    res.rel_err = rel_err;

    // clear the stats

    res.num_bv_tests  = 0;
    res.num_tri_tests = 0;

    // choose routine according to queue size
    DistanceMultiThresholdRecurse< BV_t > (res, res.T, o1, 0, o2, 0);

    Vector3D< double > u;
    u      = res.p2 - res.T.P ();
    res.p2 = res.T.R ().e ().transpose () * u;

    res.query_time_secs = timer.getTime ();

    return fcl::BVHReturnCode::BVH_OK;
}

int DistanceMultiTreshold (FCL_MultiDistanceResult& res, double threshold,
                           rw::math::Transform3D< double > T1,
                           rw::core::Ptr< fclCollisionGeometry >& o1_cg,
                           rw::math::Transform3D< double > T2,
                           rw::core::Ptr< fclCollisionGeometry >& o2_cg, double rel_err,
                           double abs_err, ProximityStrategyFCL::BV bv)
{
    using BV = ProximityStrategyFCL::BV;
    switch (bv) {
        /*case BV::AABB: FCL distance not implemented
            return FCL_DistanceMultiThreshold< rw_AABB > (
                res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
            break;*/
        /*case BV::OBB: FCL distance not implemented
            return FCL_DistanceMultiThreshold< rw_OBB > (
                res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
            break;*/
        case BV::RSS:
            return FCL_DistanceMultiThreshold< rw_RSS > (
                res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
            break;
        case BV::OBBRSS:
            return FCL_DistanceMultiThreshold< rw_OBBRSS > (
                res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
            break;
        case BV::kIOS:
            return FCL_DistanceMultiThreshold< rw_kIOS > (
                res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
            break;
            /*case BV::KDOP16: FCL distance not implemented
                FCL_DistanceMultiThreshold< rw_KDOP16 > (
                    res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
                break;*/
            /*case BV::KDOP18: FCL distance not implemented
                return FCL_DistanceMultiThreshold< rw_KDOP18 > (
                    res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
                break;*/
            /* case BV::KDOP24: FCL distance not implemented
                 return FCL_DistanceMultiThreshold< rw_KDOP24 > (
                     res, threshold, T1, o1_cg, T2, o2_cg, rel_err, abs_err);
                 break;*/
        default:
            RW_THROW ("Implementation error! Support for the chosen FCL bounding volume has not "
                      "been properly implemented!");
            break;
    }
    std::cout << "res.dist: " << res.distance << std::endl;
    return -1;
}

MultiDistanceResult& ProximityStrategyFCL::doDistances (ProximityModel::Ptr a,
                                                        const Transform3D<>& wTa,
                                                        ProximityModel::Ptr b,
                                                        const Transform3D<>& wTb, double threshold,
                                                        ProximityStrategyData& data)
{
    RW_ASSERT (a != nullptr);
    RW_ASSERT (b != nullptr);

    rw::core::Ptr< FCLProximityModel > aModel = a.cast< FCLProximityModel > ();
    rw::core::Ptr< FCLProximityModel > bModel = b.cast< FCLProximityModel > ();

    FCL_MultiDistanceResult fclResult;

    rw::proximity::DistanceMultiStrategy::Result& rwresult = data.getMultiDistanceData ();
    rwresult.clear ();

    rwresult.a = a;
    rwresult.b = b;

    fclResult.abs_err = data.abs_err;
    fclResult.rel_err = data.rel_err;

    rwresult.distance = std::numeric_limits< double >::max ();

    int geoA = -1;
    int geoB = -1;
    for (auto& ma : aModel->models) {
        geoA++;
        geoB = -1;
        for (auto& mb : bModel->models) {
            geoB++;
            fclResult.clear ();

            int code = DistanceMultiTreshold (fclResult,
                                              threshold,
                                              wTa * ma.t3d,
                                              ma.model,
                                              wTb * mb.t3d,
                                              mb.model,
                                              data.rel_err,
                                              data.abs_err,
                                              _bv);

            if (code != fcl::BVHReturnCode::BVH_OK) {
                RW_THROW ("Somthing went wrong during multidistance calculations");
            }
            typedef std::map< int, int > IdMap;
            IdMap idMap;

            for (size_t i = 0; i < fclResult.id1s.size (); i++) {
                double dist = fclResult.distances[i];
                int id      = fclResult.id1s[i];
                if (dist < rwresult.distance) {
                    rwresult.distance = dist;
                    rwresult.p1       = wTa * ma.t3d * fclResult.p1s[i];
                    rwresult.p2       = wTa * ma.t3d * fclResult.p2s[i];
                }
                IdMap::iterator res = idMap.find (id);
                if (res == idMap.end ()) {
                    idMap[id] = (int) i;
                    continue;
                }
                if (fclResult.distances[(*res).second] > dist) {
                    (*res).second = (int) i;
                }
            }

            IdMap idMap1;
            for (size_t j = 0; j < fclResult.id2s.size (); j++) {
                double dist         = fclResult.distances[j];
                int id              = fclResult.id2s[j];
                IdMap::iterator res = idMap1.find (id);
                if (res == idMap1.end ()) {
                    idMap1[id] = (int) j;
                    continue;
                }
                if (fclResult.distances[(*res).second] > dist) {
                    (*res).second = (int) j;
                }
            }

            size_t prevSize = rwresult.p1s.size ();

            size_t vsize = idMap.size () + idMap1.size ();

            rwresult.p1s.resize (prevSize + vsize);
            rwresult.p2s.resize (prevSize + vsize);
            rwresult.distances.resize (prevSize + vsize);
            rwresult.geoIdxA.resize (prevSize + vsize);
            rwresult.geoIdxB.resize (prevSize + vsize);
            rwresult.p1prims.resize (prevSize + vsize);
            rwresult.p2prims.resize (prevSize + vsize);

            size_t k = prevSize;
            for (IdMap::iterator it = idMap.begin (); it != idMap.end (); ++it, k++) {
                int idx               = (*it).second;
                rwresult.distances[k] = fclResult.distances[idx];
                rwresult.p1s[k]       = wTa * ma.t3d * fclResult.p1s[idx];
                rwresult.p2s[k]       = wTa * ma.t3d * fclResult.p2s[idx];
                rwresult.geoIdxA[k]   = geoA;
                rwresult.geoIdxB[k]   = geoB;
                rwresult.p1prims[k]   = fclResult.id1s[idx];
                rwresult.p2prims[k]   = fclResult.id2s[idx];
            }
            for (IdMap::iterator it = idMap1.begin (); it != idMap1.end (); ++it, k++) {
                int idx               = (*it).second;
                rwresult.distances[k] = fclResult.distances[idx];
                rwresult.p1s[k]       = wTa * ma.t3d * fclResult.p1s[idx];
                rwresult.p2s[k]       = wTa * ma.t3d * fclResult.p2s[idx];
                rwresult.geoIdxA[k]   = geoA;
                rwresult.geoIdxB[k]   = geoB;
                rwresult.p1prims[k]   = fclResult.id1s[idx];
                rwresult.p2prims[k]   = fclResult.id2s[idx];
            }
        }
    }
    return rwresult;
}
