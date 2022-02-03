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

#ifndef RW_PROXIMITY_PROXIMITYDATA_HPP_
#define RW_PROXIMITY_PROXIMITYDATA_HPP_

/**
 * @file rw/proximity/ProximityData.hpp
 */
#if !defined(SWIG)
#include "CollisionDetector.hpp"
#endif 
namespace rw { namespace proximity {
    class ProximityCache;

    //! @addtogroup proximity
    #if !defined(SWIG)
    //! @{
      #endif 

    /**
     * @brief Holds settings and cached data for collision detectors.
     *
     * The cache makes it possible for some algorithms to perform faster
     * detections.
     */
    class ProximityData
    {
      public:
        /**
         * @brief Default constructor.
         *
         * By default, the collision detector returns on first contact
         * with no detailed information about the collision.
         *
         * Use setCollisionQueryType to change this behaviour.
         */
        ProximityData () : _colQueryType (rw::proximity::CollisionDetector::FirstContactNoInfo) {}

#if !defined(SWIGJAVA)
        /**
         * @brief Set the type of collision query.
         *
         * The detection can perform faster if it is allowed to return
         * after detecting the first collision. Alternatively, it is
         * possible to detect all collisions if required.
         *
         * @param qtype [in] the query type.
         * @see rw::proximity::CollisionDetector::QueryType
         */

         #endif 
        void setCollisionQueryType (rw::proximity::CollisionDetector::QueryType qtype) { _colQueryType = qtype; }
#if !defined(SWIGJAVA)
        /**
         * @brief Get the collision query type.
         * @return the query type.
         * @see CollisionDetector::QueryType
         */

         #endif 
        rw::proximity::CollisionDetector::QueryType getCollisionQueryType () const { return _colQueryType; }
#if !defined(SWIGJAVA)
        /**
         * @brief Detailed information about the collision.
         * @note This data is only available for some collision query types.
         * @see rw::proximity::CollisionDetector::QueryResult
         */

         #endif 
        rw::proximity::CollisionDetector::QueryResult _collisionData;

        /**
         * @brief Cached data used by the collision detector to speed up
         * consecutive queries.
         */
        rw::core::Ptr< ProximityCache > _cache;

      private:
        rw::proximity::CollisionDetector::QueryType _colQueryType;
    };
#if !defined(SWIG)
    //! @}
    #endif 
}}    // namespace rw::proximity

#endif /* PROXIMITYDATA_HPP_ */
