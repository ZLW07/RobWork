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

#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYFCL_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYFCL_HPP

/**
 * @file ProximityStrategyFCL.hpp
 */

#include <rw/core/Ptr.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

#include <fcl/config.h>
#if (FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION < 6)
#define FCL_VERSION_LESS_THEN_0_6_0 true
#endif

#include <string>
#include <utility>
#include <vector>

#if (FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION < 6)
#define FCL_VERSION_LESS_THEN_0_6_0 true
#endif

namespace fcl {
#ifdef FCL_VERSION_LESS_THEN_0_6_0
class CollisionGeometry;
class CollisionResult;
class CollisionRequest;
class DistanceRequest;
class DistanceResult;
#else
template< typename S = double > class CollisionGeometry;
template< typename S = double > struct CollisionResult;
template< typename S = double > struct CollisionRequest;
template< typename S = double > struct DistanceRequest;
template< typename S = double > struct DistanceResult;
#endif
}    // namespace fcl

namespace rwlibs { namespace proximitystrategies {
#ifdef FCL_VERSION_LESS_THEN_0_6_0

    //! @brief Type of internal collision result.
    using fclCollisionResult = fcl::CollisionResult;

    //! @brief Type of internal collision request.
    using fclCollisionRequest = fcl::CollisionRequest;

    //! @brief Type of internal distance request.
    using fclDistanceRequest = fcl::DistanceRequest;

    //! @brief Type of internal distance result.
    using fclDistanceResult = fcl::DistanceResult;

    //! @brief Type of internal collision Geometry.
    using fclCollisionGeometry = fcl::CollisionGeometry;

#else
    //! @brief Type of internal collision result.
    using fclCollisionResult = fcl::CollisionResult< double >;

    //! @brief Type of internal collision request.
    using fclCollisionRequest = fcl::CollisionRequest< double >;

    //! @brief Type of internal distance request.
    using fclDistanceRequest = fcl::DistanceRequest< double >;

    //! @brief Type of internal distance result.
    using fclDistanceResult = fcl::DistanceResult< double >;

    //! @brief Type of internal collision Geometry.
    using fclCollisionGeometry = fcl::CollisionGeometry< double >;
#endif

    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief This is a strategy wrapper for the Flexible Collision Library (FCL)
     *
     * For further information check out https://github.com/flexible-collision-library/fcl
     */
    class ProximityStrategyFCL : public rw::proximity::CollisionStrategy,
                                 public rw::proximity::DistanceStrategy,
                                 public rw::proximity::DistanceMultiStrategy
    {
      public:
        //! @brief Smart pointer type for FCL Proximity strategy.
        typedef rw::core::Ptr< ProximityStrategyFCL > Ptr;

        //! @brief Type of internal collision geometry.
        typedef rw::core::Ptr< fclCollisionGeometry > FCLBVHModelPtr;

        //! @brief Datatype to hold the FCL bounding volume and related geometrical data.
        struct FCLModel
        {
            /**
             * @brief Create new holder for internal collision geometry information.
             * @param geoId [in] id of the geometry.
             * @param transform [in] transform of the geometry.
             * @param model [in] the internal model of the collision geometry.
             */
            FCLModel (rw::core::Ptr< rw::geometry::Geometry > geo,
                      const rw::math::Transform3D<>& transform, FCLBVHModelPtr model) :
                geo (geo),
                t3d (transform), model (model)
            { /* Empty */
            }
            //! @brief Identifier for the geometry.
            rw::core::Ptr< rw::geometry::Geometry > geo;
            //! @brief Location of the geometry.
            rw::math::Transform3D<> t3d;
            //! @brief Using fcl::CollisionGeometry as the type of the model, to allow holding all
            //! the different fcl::BVHModel{bv type} types.
            FCLBVHModelPtr model;
        };

        //! @brief Type for list of proximity models.
        typedef std::vector< FCLModel > FCLModelList;

        //! @brief Datatype to hold the proximity models
        struct FCLProximityModel : public rw::proximity::ProximityModel
        {
            /**
             * @brief Constructor.
             * @param owner [in] the strategy owning this model.
             */
            FCLProximityModel (ProximityStrategy* owner) : ProximityModel (owner)
            {
                /* Nothing specific */
            }
            //! @brief Models holding the internal collision geometry.
            FCLModelList models;
        };

        //! @brief Supported bounding volumes
        enum class BV {
            AABB,      //!< Axis-Aligned Bounding Boxes
            OBB,       //!< Oriented Bounding Boxes
            RSS,       //!< Rectangle Swept Spheres
            OBBRSS,    //!< Mix of OBB and RSS
            kIOS,      //!< Bounding volume as the intersection of a set of spheres
            KDOP16,    //!< Discrete Oriented Polytope
            KDOP18,    //!< Discrete Oriented Polytope
            KDOP24     //!< Discrete Oriented Polytope
        };

        /**
         * @brief Constructor
         * @param bv [in] the bounding volume type to use.
         */
        ProximityStrategyFCL (BV bv = BV::RSS);

        /**
         * @brief Destructor
         */
        virtual ~ProximityStrategyFCL ();

        //// interface of ProximityStrategy
        //! @copydoc rw::proximity::ProximityStrategy::createModel
        virtual rw::proximity::ProximityModel::Ptr createModel ();

        //! @copydoc rw::proximity::ProximityStrategy::destroyModel
        void destroyModel (rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::ProximityStrategy::addGeometry(rw::proximity::ProximityModel*
         * model, const rw::geometry::Geometry& geom)
         *
         * @throws Exception when a bounding volume type has been chosen that is not supported.
         */
        bool addGeometry (rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

        /**
         * @copydoc rw::proximity::ProximityStrategy::addGeometry(ProximityModel* model,
         * rw::core::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false)
         *
         * @throws Exception when a bounding volume type has been chosen that is not supported.
         */
        bool addGeometry (rw::proximity::ProximityModel* model,
                          rw::core::Ptr< rw::geometry::Geometry > geom, bool forceCopy = false);

        //! @copydoc rw::proximity::ProximityStrategy::removeGeometry
        bool removeGeometry (rw::proximity::ProximityModel* model, const std::string& geomId);

        //! @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
        std::vector< std::string > getGeometryIDs (rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::ProximityStrategy::getGeometrys
         */
        std::vector< rw::core::Ptr< rw::geometry::Geometry > >
        getGeometrys (rw::proximity::ProximityModel* model);

        //! @copydoc rw::proximity::ProximityStrategy::clear
        void clear ();

        //// Interface of CollisionStrategy
        //! @copydoc rw::proximity::CollisionStrategy::doInCollision
        bool doInCollision (rw::proximity::ProximityModel::Ptr a,
                            const rw::math::Transform3D<>& wTa,
                            rw::proximity::ProximityModel::Ptr b,
                            const rw::math::Transform3D<>& wTb,
                            rw::proximity::ProximityStrategyData& data);

        /**
         * @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
         *
         * @note Not implemented as nothing appears to be using this functionality
         */
        void getCollisionContacts (std::vector< rw::proximity::CollisionStrategy::Contact >& contacts,
                                   rw::proximity::ProximityStrategyData& data);

        //// Interface of DistanceStrategy
        //! @copydoc rw::proximity::DistanceStrategy::doDistance
        rw::proximity::DistanceStrategy::Result&
        doDistance (rw::proximity::ProximityModel::Ptr a, const rw::math::Transform3D<>& wTa,
                    rw::proximity::ProximityModel::Ptr b, const rw::math::Transform3D<>& wTb,
                    class rw::proximity::ProximityStrategyData& data);

        //// Interface of DistanceMultiStrategy
        //! @copydoc rw::proximity::DistanceMultiStrategy::doDistance
        rw::proximity::DistanceMultiStrategy::Result&
        doDistances (rw::proximity::ProximityModel::Ptr a, const rw::math::Transform3D<>& wTa,
                     rw::proximity::ProximityModel::Ptr b, const rw::math::Transform3D<>& wTb,
                     double tolerance, class rw::proximity::ProximityStrategyData& data);
        //// End of interfaces

        /**
         * @brief Set the bounding volume
         * @param bv [in] new bounding volume type.
         */
        void setBV (const BV& bv) { _bv = bv; }

        /**
         * @brief Get the bounding volume
         * @return the type of bounding volume used.
         */
        BV getBV () { return _bv; }

        /**
         * @brief Get access to the CollisionRequest that is used with FCL collision query
         *
         * See the fcl/collision_data.h for the data structure, and look through their documentation
         * for specifics on what solvers that can be chosen.
         */
        fclCollisionRequest& getCollisionRequest ();

        /**
         * @brief Get access to the DistanceRequest that is used with FCL distance query
         *
         * See the fcl/collision_data.h for the data structure, and look through their documentation
         * for specifics on what solvers that can be chosen.
         *
         * @note The rel_err and abs_err fields will be overwritten with the values that are
         * supplied in the rw::proximity::ProximityStrategyData
         */
        fclDistanceRequest& getDistanceRequest ();

        /**
         * @brief Specify if FCL results should be collected
         */
        void setCollectFCLResults (bool enable) { _collectFCLResults = enable; }

        /**
         * @brief Get whether FCL results should be collected or not
         */
        bool getCollectFCLResults () { return _collectFCLResults; }

        /**
         * @brief Get access to the collected FCL collision results
         */
        std::vector< fclCollisionResult >& getCollisionResults () { return _fclCollisionResults; }

        /**
         * @brief Get access to the collected FCL collision result
         *
         * @throws std::out_of_range exception if index is not within the bounds.
         */
        fclCollisionResult& getCollisionResult (std::size_t index);

        /**
         * @brief Get access to the collected FCL distance result
         */
        fclDistanceResult& getDistanceResult ();

        std::pair< rw::math::Vector3D<>, rw::math::Vector3D<> >
        getSurfaceNormals (rw::proximity::DistanceMultiStrategy::Result& res, int idx);

      private:
        template< typename BV >
        bool addGeometry (rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);
        template< typename BV >
        bool addGeometry (rw::proximity::ProximityModel* model,
                          rw::core::Ptr< rw::geometry::Geometry > geom, bool forceCopy);

      private:
        BV _bv;
        fclCollisionRequest* const _fclCollisionRequest;
        fclDistanceRequest* const _fclDistanceRequest;

        bool _collectFCLResults;
        std::vector< fclCollisionResult > _fclCollisionResults;
        fclDistanceResult* _fclDistanceResult;
    };
}}    // namespace rwlibs::proximitystrategies

#endif    // include guard
