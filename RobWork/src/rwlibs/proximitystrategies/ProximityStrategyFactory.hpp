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

#ifndef RWLIBS_PROXIMITYSTRATEGIES_ProximityStrategyFactory_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_ProximityStrategyFactory_HPP

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {

    /**
     * @brief Factory class that enables constructing collision strategies
     */
    class ProximityStrategyFactory
    {
      public:
        /**
         * @brief Get a list of all available CollisionStrategiy ID's
         * @return A vector of ID's
         */
        static std::vector< std::string > getCollisionStrategyIDs ();

        /**
         * @brief function to create a default available collision strategy
         * @return NULL if no collision strategies are available else a Ptr to a
         * collision strategy
         */
        static rw::proximity::CollisionStrategy::Ptr makeDefaultCollisionStrategy ();

        /**
         * @brief function to create a collision strategy from an ID
         * @param id [in] the id of the collision strategy
         * @return NULL if the \b id dosn't match an available collision strategies else a Ptr to
         * the collision strategy
         */
        static rw::proximity::CollisionStrategy::Ptr makeCollisionStrategy (const std::string& id);

        /**
         * @brief Get a list of all available DistanceStrategiy ID's
         * @return A vector of ID's
         */

        static std::vector< std::string > getDistanceStrategyIDs ();

        /**
         * @brief function to create a default available distance strategy
         * @return NULL if no distancestrategies are available else a Ptr to a
         * distance strategy
         */
        static rw::proximity::DistanceStrategy::Ptr makeDefaultDistanceStrategy ();

        /**
         * @brief function to create a distance strategy from an ID
         * @param id [in] the id of the distance strategy
         * @return NULL if the \b id dosn't match an available distance trategies else a Ptr to the
         * distance strategy
         */
        static rw::proximity::DistanceStrategy::Ptr makeDistanceStrategy (const std::string& id);

        /**
         * @brief Get a list of all available DistanceMultiStrategiy ID's
         * @return A vector of ID's
         */
        static std::vector< std::string > getDistanceMultiStrategyIDs ();

        /**
         * @brief function to create a default available distance multi strategy
         * @return NULL if no distance multi strategies are available else a Ptr to a
         * distance multi strategy
         */
        static rw::proximity::DistanceMultiStrategy::Ptr makeDefaultDistanceMultiStrategy ();

        /**
         * @brief function to create a distance multi strategy from an ID
         * @param id [in] the id of the distance multi strategy
         * @return NULL if the \b id dosn't match an available distance multi strategies else a Ptr
         * to the distnace multi strategy
         */
        static rw::proximity::DistanceMultiStrategy::Ptr
        makeDistanceMultiStrategy (const std::string& id);
    };

}}    // namespace rwlibs::proximitystrategies

#endif    // end include guard
