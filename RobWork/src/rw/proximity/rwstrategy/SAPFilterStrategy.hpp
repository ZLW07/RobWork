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

#ifndef RW_PROXIMITY_SAPFilterStrategy_HPP_
#define RW_PROXIMITY_SAPFilterStrategy_HPP_

#if !defined(SWIG)
#include "ProximityFilterStrategy.hpp"
#include "ProximitySetup.hpp"
#endif 
namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rw { namespace proximity {

    /**
     * @brief This is a Sweep-And-Prune based filter strategy (broadphase strategy).
     *
     */
    class SAPFilterStrategy : public ProximityFilterStrategy
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< SAPFilterStrategy > Ptr;

      private:
        /**
         * @brief the proximity cache of the basic filter
         */
        struct Cache : public ProximityCache
        {
          public:
            Cache (void* owner) : ProximityCache (owner){};
            size_t size () const { return 0; };
            void clear (){};
        };

        struct Filter : public ProximityFilter
        {
          public:
            Filter (kinematics::FramePairSet::iterator front,
                    kinematics::FramePairSet::iterator end) :
                _front (front),
                _end (end)
            {}

            void pop () { ++_front; };

            kinematics::FramePair frontAndPop ()
            {
                kinematics::FramePair res = *_front;
                pop ();
                return (res);
            }

            rw::kinematics::FramePair front () { return *_front; };

            bool isEmpty () { return _front == _end; };

          private:
            kinematics::FramePairSet::iterator _front, _end;
        };

      public:
        /**
         * @brief constructor - the ProximitySetup will be extracted from
         * the workcell description if possible.
         *
         * @param workcell [in] the workcell.
         */
        SAPFilterStrategy (rw::core::Ptr< rw::models::WorkCell > workcell);

        /**
         * @brief constructor - constructs frame pairs based on the \b setup
         * @param workcell [in] the workcell
         * @param setup [in] the ProximitySetup describing exclude/include relations
         */
        SAPFilterStrategy (rw::core::Ptr< rw::models::WorkCell > workcell,
                           const ProximitySetup& setup);

        //! @brief destructor
        virtual ~SAPFilterStrategy (){};

        //////// interface inherited from BroadPhaseStrategy

        //! @copydoc ProximityFilterStrategy::reset
        virtual void reset (const rw::kinematics::State& state);

        //! @copydoc ProximityFilterStrategy::createProximityCache
        virtual rw::core::Ptr<rw::proximity::ProximityCache> createProximityCache ()
        {
            return rw::core::ownedPtr (new Cache (this));
        }

        //! @copydoc ProximityFilterStrategy::update
        virtual rw::core::Ptr<rw::proximity::ProximityFilter> update (const rw::kinematics::State& state);

        //! @copydoc ProximityFilterStrategy::createProximityCache
        virtual rw::core::Ptr<rw::proximity::ProximityFilter> update (const rw::kinematics::State& state,
                                             rw::core::Ptr<rw::proximity::ProximityCache> data);

        /**
         * @copydoc ProximityFilterStrategy::getProximitySetup
         */
        ProximitySetup& getProximitySetup ();

        /**
         * @brief Adds geometry associated to frame
         * @param frame [in] Frame which has the geometry associated
         */
        virtual void addGeometry (rw::kinematics::Frame* frame, const rw::geometry::Geometry::Ptr);

        /**
         * @brief Removes the geometric model with name \b rw::geometry::Geometry::Ptr and which is
         * associated with \b frame.
         *
         * @param frame [in] Frame which has the geometry associated
         */
        virtual void removeGeometry (rw::kinematics::Frame* frame,
                                     const rw::geometry::Geometry::Ptr);

        /**
         * @brief Removes the geometric model with name \b geometryId and which is associated with
         * \b frame.
         *
         * @param frame [in] Frame which has the geometry associated
         * @param geometryId [in] Name of geometry
         */
        virtual void removeGeometry (rw::kinematics::Frame* frame, const std::string& geometryId);

        /**
         * @copydoc ProximityFilterStrategy::addRule
         */
        virtual void addRule (const ProximitySetupRule& rule);

        /**
         * @copydoc ProximityFilterStrategy::removeRule
         */
        virtual void removeRule (const ProximitySetupRule& rule);

      private:
        rw::core::Ptr< rw::models::WorkCell > _workcell;
        ProximitySetup _psetup;
        kinematics::FramePairSet _collisionPairs;

        kinematics::FrameMap< std::vector< std::string > > _frameToGeoIdMap;

        void applyRule (const ProximitySetupRule& rule,
                        rw::core::Ptr< rw::models::WorkCell > workcell,
                        rw::kinematics::FramePairSet& result);
        void initialize ();
        void initializeCollisionFramePairs (const rw::kinematics::State& state);
    };

#ifdef RW_USE_DEPREACTED
    typedef rw::core::Ptr< SAPFilterStrategy > SAPFilterStrategyPtr;
#endif
}}    // namespace rw::proximity

#endif /* SAPFilterStrategy_HPP_ */
