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

#ifndef RW_PROXIMITY_COLLISIONDETECTOR_HPP
#define RW_PROXIMITY_COLLISIONDETECTOR_HPP

/**
 * @file CollisionDetector.hpp
 *
 * \copydoc rw::proximity::CollisionDetector
 */

#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/proximity/ProximityCalculator.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#include <utility>
#include <vector>
#endif

namespace rw { namespace kinematics {
    class Frame;
    class State;
}}    // namespace rw::kinematics

namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rw { namespace proximity {
    class CollisionStrategy;
    class ProximityData;
    /** @addtogroup proximity */
    /*@{*/

    /**
     @brief The CollisionDetector implements an efficient way of checking a
     complete frame tree for collisions.

     It relies on a BroadPhaseDetector to do initial filtering which removes obviously not
     colliding frame pairs.

     After the filtering the remaining frame pairs are tested for collision using an
     CollisionStrategy which is a narrow phase collision detector.

     The collision detector does not dictate a specific detection
     strategy or algorithm, instead it relies on the CollisionStrategy interface for
     the actual collision checking between two frames.

     @note The collision detector is not thread safe and as such should not be used by multiple
     threads at a time.
     */
    class CollisionDetector : public rw::proximity::ProximityCalculator< rw::proximity::CollisionStrategy >
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< CollisionDetector > Ptr;
        //! @brief smart pointer type to this const class
        typedef rw::core::Ptr< const CollisionDetector > CPtr;

#if !defined(SWIG)
        //! @brief types of collision query
        typedef enum {
            AllContactsFullInfo,     //! find all collisions and return full collision information
                                     //! eg. CollisionStrategy::AllContact
            AllContactsNoInfo,       //! find all collisions but without collision information eg.
                                     //! CollisionStrategy::FirstContact
            FirstContactFullInfo,    //! return on first contact and include full collision
                                     //! information eg. CollisionStrategy::AllContact
            FirstContactNoInfo       //! return on first collision but without collision information
                                     //! eg. CollisionStrategy::FirstContact
        } QueryType;
        #else 
        typedef int QueryType;
#endif
        /**
         * @brief result of a collision query
         */
        struct QueryResult
        {
            /**
             * @brief convert the framePair set to FramePairVector
             * @return collidingFrames as frame pair vector
             */
            std::vector< std::pair< rw::kinematics::Frame*, rw::kinematics::Frame* > >
            getFramePairVector ()
            {
                return std::vector< std::pair< rw::kinematics::Frame*, rw::kinematics::Frame* > > (
                    this->collidingFrames.begin (), this->collidingFrames.end ());
            }
            //! the frames that are colliding
            rw::kinematics::FramePairSet collidingFrames;

            //! for keeping track of all collision data: AllContactsFullInfo, FirstContactNoInfo
            std::vector< ProximityStrategyData > _fullInfo;
        };

        /**
         * @brief Collision detector for a workcell with only broad-phase collision checking.
         *
         * The default collision setup stored in the workcell is used for
         * broad phase collision filtering as a static filter list.
         *
         * Notice that no narrow phase checking is performed.
         * If broad-phase filter returns any frame-pairs, this will be taken as a collision.
         *
         * @param workcell [in] the workcell.
         */
        CollisionDetector (rw::core::Ptr< rw::models::WorkCell > workcell);

        /**
         * @brief Collision detector for a workcell.
         *
         * The collision detector is initialized with the \b strategy .
         * Notice that the collision detector will create and store models inside the \b strategy .
         *
         * The default collision setup stored in the workcell is used for
         * broad phase collision filtering as a static filter list.
         *
         * @param workcell [in] the workcell.
         * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have
         * models added to it.
         */
        CollisionDetector (rw::core::Ptr< rw::models::WorkCell > workcell,
                           rw::core::Ptr<rw::proximity::CollisionStrategy> strategy);

        /**
         * @brief Collision detector for a workcell.
         * Collision checking is done for the provided collision setup alone.
         *
         * @param workcell [in] the workcell.
         * @param strategy [in/out] the strategy for narrow-phase checking. The strategy will have
         * models added to it.
         * @param filter [in] proximity filter used to cull or filter frame-pairs that are obviously
         * not colliding
         */
        CollisionDetector (rw::core::Ptr< rw::models::WorkCell > workcell,
                           rw::core::Ptr<rw::proximity::CollisionStrategy> strategy,
                           rw::core::Ptr< ProximityFilterStrategy > filter);

        /**
         * @brief Check the workcell for collisions.
         *
         * @param state [in] The state for which to check for collisions.
         * @param result [out] If non-NULL, the pairs of colliding frames are
         * inserted in \b result.
         * @param stopAtFirstContact [in] If \b result is non-NULL and \b
         * stopAtFirstContact is true, then only the first colliding pair is
         * inserted in \b result. By default all colliding pairs are inserted.
         *
         * @return true if a collision is detected; false otherwise.
         */
        bool inCollision (const kinematics::State& state, QueryResult* result = 0,
                          bool stopAtFirstContact = false) const;

        /**
         @brief Check the workcell for collisions.
         @param state [in] The state for which to check for collisions.
         @param data [in/out] Defines parameters for the collision check, the results and also
         enables caching inbetween calls to incollision
         @return true if a collision is detected; false otherwise.
         */
        bool inCollision (const kinematics::State& state, rw::proximity::ProximityData& data) const;

        /**
         * @brief Check the workcell for collisions.
         *
         * @param state [in] The state for which to check for collisions.
         * @param result [out] Where to store pairs of colliding frames.
         * @param stopAtFirstContact [in] If \b result is non-NULL and \b
         * stopAtFirstContact is true, then only the first colliding pair is
         * inserted in \b result. By default all colliding pairs are inserted.
         *
         * @return true if a collision is detected; false otherwise.
         */
        bool inCollision (
            const rw::kinematics::State& state,
            std::vector< std::pair< rw::kinematics::Frame*, rw::kinematics::Frame* > >& result,
            bool stopAtFirstContact = false)
        {
            rw::proximity::CollisionDetector::QueryResult data;
            bool success;
            success = this->inCollision (state, &data, stopAtFirstContact);

            result = std::vector< std::pair< rw::kinematics::Frame*, rw::kinematics::Frame* > > (
                data.collidingFrames.begin (), data.collidingFrames.end ());

            return success;
        }
        /**
         * @brief Get the narrow-phase collision strategy.
         * @return the strategy if set, otherwise NULL.
         */
        rw::core::Ptr<rw::proximity::CollisionStrategy> getCollisionStrategy () const { return getStrategy (); }

        /**
         * @brief Add Geometry associated to \b frame
         *
         * The current shape of the geometry is copied, hence later changes to \b geometry has no
         * effect
         *
         * @param frame [in] Frame to associate geometry to
         * @param geometry [in] Geometry to add
         */
        void addGeometry (rw::kinematics::Frame* frame,
                          const rw::core::Ptr< rw::geometry::Geometry > geometry);

        /**
         * @brief Removes geometry from CollisionDetector
         *
         * The id of the geometry is used to match the collision model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometry [in] Geometry with the id to be removed
         */
        void removeGeometry (rw::kinematics::Frame* frame,
                             const rw::core::Ptr< rw::geometry::Geometry > geometry);

        /**
         * @brief Removes geometry from CollisionDetector
         *
         * The \b geometryId is used to match the collision model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometryId [in] Id of geometry to be removed
         */
        void removeGeometry (rw::kinematics::Frame* frame, const std::string geometryId);

        /**
         * @brief return the ids of all the geometries of this frames.
         */
        std::vector< std::string > getGeometryIDs (rw::kinematics::Frame* frame);

        /**
         * @brief Returns whether frame has an associated geometry with \b geometryId.
         * @param frame [in] Frame in question
         * @param geometryId [in] Id of the geometry
         */
        bool hasGeometry (rw::kinematics::Frame* frame, const std::string& geometryId);

        /**
         * @brief Get the geometry from its ID
         * @param ID [in] the ID of the geometry
         * @return Pointer to the geometry
         */
        rw::core::Ptr< rw::geometry::Geometry > getGeometry (rw::kinematics::Frame* frame,
                                                             const std::string& geometryId);

        static rw::core::Ptr< rw::proximity::CollisionDetector >
        make (rw::core::Ptr< rw::models::WorkCell > workcell,
              rw::core::Ptr< rw::proximity::CollisionStrategy > strategy)
        {
            return rw::core::ownedPtr (new CollisionDetector (workcell, strategy));
        }

      private:
#if __cplusplus < 201103L
      private:
        CollisionDetector (const CollisionDetector&);
        CollisionDetector& operator= (const CollisionDetector&);
#endif
    };

    /*@}*/
}}    // namespace rw::proximity

#endif    // end include guard
