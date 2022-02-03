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

#ifndef RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_
#define RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_

/**
 * @file rw/proximity/ProximityStrategyData.hpp
 */

#if !defined(SWIG)
#include "CollisionStrategy.hpp"
#include "DistanceMultiStrategy.hpp"
#include "DistanceStrategy.hpp"
#include "ProximityCache.hpp"
#include "ProximityModel.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>

#include <float.h>
#include <utility>
#include <vector>
#endif 
namespace rw { namespace proximity {
    //! @addtogroup proximity
    // @{

    // for backward compatability
    typedef rw::proximity::CollisionStrategy::Result CollisionResult;
    typedef rw::proximity::DistanceStrategy::Result DistanceResult;
    typedef DistanceMultiStrategy::Result MultiDistanceResult;

    /***
     * @brief A generic object for containing data that is essential in
     * proximity queries between two ProximityModels.
     *
     * The ProximityStrategyData object is used for Collision queries, tolerance and distance queries
     * between two ProximityModels. example: collision result, cached variables for faster collision
     * detection,
     *
     */
    class ProximityStrategyData
    {
      public:
        typedef rw::core::Ptr< ProximityStrategyData > Ptr;
        typedef std::vector< ProximityStrategyData > List;
        typedef rw::core::Ptr< std::vector< ProximityStrategyData > > PtrList;

        /**
         * @brief Create Empty ProximityStrategyData
         */
        ProximityStrategyData () :
            rel_err (0), abs_err (0), _colQueryType (rw::proximity::CollisionStrategy::FirstContact),
            _collides (false)
        {
            _collisionData.clear ();
            _distanceData.clear ();
            _multiDistanceData.clear ();
            _multtiDistanceTolerance = DBL_MAX;
        }

        /**
         * @brief Copy Constructor
         */
        ProximityStrategyData (const ProximityStrategyData& data) :
            rel_err (data.rel_err), abs_err (data.abs_err), _colQueryType (data._colQueryType),
            _collides (data._collides), _collisionData (data._collisionData),
            _distanceData (data._distanceData), _multiDistanceData (data._multiDistanceData),
            _multtiDistanceTolerance (data._multtiDistanceTolerance), _cache (data._cache)
        {}

        /**
         * @brief Get the underlying cache
         * @return pointer to cache
         */
        rw::core::Ptr<rw::proximity::ProximityCache>& getCache () { return _cache; }

        // CollisionData interface
        /**
         * @brief get the result from the collision check
         * @return Result of Collision strategy if available
         */
        rw::proximity::CollisionStrategy::Result& getCollisionData () { return _collisionData; }

        /**
         * @brief get the result from the collision check
         * @return Result of Collision strategy if available
         */
        const rw::proximity::CollisionStrategy::Result& getCollisionData () const { return _collisionData; }

        /**
         * @brief get the the colliding frames
         * @return the cooliding frames, if in collision else a pair of null
         */
        std::pair< rw::core::Ptr< rw::kinematics::Frame >,
                   rw::core::Ptr< rw::kinematics::Frame > >
        getColidingFrames ()
        {
            if (!_collisionData.a.isNull () && !_collisionData.b.isNull ()) {
                return std::make_pair (
                    rw::core::Ptr< rw::kinematics::Frame > (_collisionData.a->getFrame ()),
                    rw::core::Ptr< rw::kinematics::Frame > (_collisionData.b->getFrame ()));
            }
            return std::make_pair (rw::core::Ptr< rw::kinematics::Frame > (NULL),
                                   rw::core::Ptr< rw::kinematics::Frame > (NULL));
        }

        /**
         * @brief was collision check in collision
         * @return true if in collision
         */
        bool& inCollision () { return _collides; }

        /**
         * @brief set the Collision Query type
         * @param qtype [in] the used Query type
         */
        void setCollisionQueryType (rw::proximity::CollisionStrategy::QueryType qtype) { _colQueryType = qtype; }

        /**
         * @brief Get the used Collision Query type
         * @return Querytype
         */
        rw::proximity::CollisionStrategy::QueryType getCollisionQueryType () const { return _colQueryType; }

        // Distance query interfaces
        /**
         * @brief get The result of a distance query
         * @return result of a distance query
         */
        rw::proximity::DistanceStrategy::Result& getDistanceData () { return _distanceData; }

        // Distance query interfaces
        /**
         * @brief get The result of a distance query
         * @return result of a distance query
         */
        const rw::proximity::DistanceStrategy::Result& getDistanceData () const { return _distanceData; }

        // For Multi distance interface
        /**
         * @brief get The result of a multi distance query
         * @return result of a distance query
         */
        DistanceMultiStrategy::Result& getMultiDistanceData () { return _multiDistanceData; }

        /**
         * @brief get The result of a multi distance query
         * @return result of a distance query
         */
        const DistanceMultiStrategy::Result& getMultiDistanceData () const
        {
            return _multiDistanceData;
        }

        /**
         * @brief get the tolerance used to treshold which distances are recorded and which are not.
         * point pairs that are closer than tolerance will be included in the
         * result.
         * @return The set tolerance
         */
        double getMultiDistanceTolerance () { return _multtiDistanceTolerance; }

        /**
         * @brief set the tolerance used to treshold which distances are recorded and which are not.
         * point pairs that are closer than tolerance will be included in the
         * result.
         * @param tolerance [in] set the stored tolerance
         */
        void setMultiDistanceTolerance (double tolerance) { _multtiDistanceTolerance = tolerance; }

        //! @brief relative acceptable error
        double rel_err;
        //! @brief absolute acceptable error
        double abs_err;

      private:
        // Collision data
        rw::proximity::CollisionStrategy::QueryType _colQueryType;
        bool _collides;
        rw::proximity::CollisionStrategy::Result _collisionData;

        // Distance data
        rw::proximity::DistanceStrategy::Result _distanceData;
        DistanceMultiStrategy::Result _multiDistanceData;
        double _multtiDistanceTolerance;

        //! @brief proximity cache
        rw::core::Ptr<rw::proximity::ProximityCache> _cache;
    };
    // @}
}}    // namespace rw::proximity

#endif /* RW_PROXIMITY_PROXIMITYSTRATEGYDATA_HPP_ */
