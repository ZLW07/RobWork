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

#ifndef RW_PROXIMITY_PROXIMITYCALCULATOR_HPP
#define RW_PROXIMITY_PROXIMITYCALCULATOR_HPP

#if !defined(SWIG)
#include <rw/common/Timer.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/State.hpp>

#include <utility>
#include <vector>
#endif 

namespace rw { namespace kinematics {
    class Frame;
}}    // namespace rw::kinematics
namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rw { namespace geometry {
    class Geometry;
}}    // namespace rw::geometry

namespace rw { namespace proximity {
    class ProximityFilterStrategy;
    class ProximitySetupRule;
    class ProximityStrategyData;
    class ProximityModel;
    class CollisionSetup;

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The Proximity calculator implements an efficient and standardized way of using the
     * following proximity strategies:
     *
     * CollisionStrategy
     * DistanceStrategy
     * MultiDistanceStrategy
     *
     * The Calculate function is designed to fit the chosen strategy individually implementing a
     * fitting aproach for calculating the respective proximity.
     *
     * \b The \b CollisionDetector
     * It relies on a BroadPhaseDetector to do initial filtering which removes obviously not
     * colliding frame pairs.
     *
     * After the filtering the remaining frame pairs are tested for collision using an
     * CollisionStrategy which is a narrow phase collision detector.
     *
     * The Proximity calculator does not dictate a specific detection
     * strategy or algorithm, instead it relies on the CollisionStrategy interface for
     * the actual collision checking between two frames.
     *
     * \b Distance \b and \b MultiDistance \b Calculator
     * A list of frame pairs is contained within the Proximity calculator,
     * that specifies which frames are to be checked against each other.
     * The method of used for distance calculation relies on the DistanceStrategy
     * chosen.
     */
    template< class T > class ProximityCalculator
    {
      public:
        //! @brief the strategy used for detection
        typedef T Strategy;
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< ProximityCalculator > Ptr;
        //! @brief smart pointer type to this const class
        typedef rw::core::Ptr< const ProximityCalculator > CPtr;
        //! @brief the type used to store results in.
        typedef rw::core::Ptr< std::vector< ProximityStrategyData > > ResultType;

        /**
         * @brief Proximity calculations for a given tree, collision setup and
         * primitive Proximity calculator. Uses proximity strategy given by the workcell.
         * @param root [in] - the root of the Frame tree. must be non-NULL. No ownership of the
         * pointer is taken
         * @param workcell [in] - the workcell to do the proximity calculations in.
         * @param strategy [in] - the primitive strategy of proximity calculations. must be
         * non-NULL.
         * @param initial_state [in] - the work cell state to use for the
         * initial traversal of the tree.
         */
        ProximityCalculator (rw::core::Ptr< rw::kinematics::Frame > root,
                             rw::core::Ptr< rw::models::WorkCell > workcell,
                             rw::core::Ptr< Strategy > strategy,
                             const rw::kinematics::State& initial_state);

        /**
         * @brief Construct proximity calculator for a WorkCell with an associated
         *  proximity strategy.
         *
         * The ProximityCalculator extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * The ProximityCalculator is initialized with the \b strategy .
         * Notice that the ProximityCalculator will create and store models inside the \b strategy .
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the ProximityStrategy to use
         */
        ProximityCalculator (rw::core::Ptr< rw::models::WorkCell > workcell,
                             rw::core::Ptr< Strategy > strategy);
        
#if __cplusplus >= 201103L
        //! @brief Copy constructor is non-existent. Copying is not possible!
        ProximityCalculator (const ProximityCalculator&) = delete;

        //! @brief Assignment operator is non-existent. Copying is not possible!
        ProximityCalculator& operator= (const ProximityCalculator&) = delete;
#endif

        /**
         * @brief Performece the Proximity calculation based on the chosen strategy type.
         * As the varius strategies usese differenct settings all settings will be extracted
         * from \b settings. If more then the default result is needed (first collision or shortest
         * distance) \b result can given to get the extra info.
         * @param state [in] The state the proximity calculation should be done in.
         * @param settings [in] The settings used for the calculations. Different settings are used
         * for different ProximityStrategies:
         *
         * For CollisionStrategy the Collision Query Type is used. if not given only first collision
         * is detected
         *
         * For DistanceStrategy no settings are used and it is expected to be null, otherwise an
         * exception is thrown.
         *
         * For DistanceMultiStrategy the tolerance is used which is the maximum distance allowed for
         * the result to be recorded. if not given the tolerance is set to the largest finite double
         *
         * @param result [in/out] Defines parameters for the ProximityCalculation, stores the
         * results and also enables caching inbetween calls.
         * @return If no result is available an empty ProximityStrategyData is returned. else for
         * Collisions the first contact is returned and for distance the shortest distance is
         * returned
         */
        ProximityStrategyData
        calculate (const rw::kinematics::State& state,
                   rw::core::Ptr< ProximityStrategyData > settings               = NULL,
                   rw::core::Ptr< std::vector< ProximityStrategyData > > results = NULL);

        /**
         * @brief The Proximity Filter strategy of the ProximityCalculator.
         */
        rw::core::Ptr< ProximityFilterStrategy > getProximityFilterStrategy () const
        {
            return _proxFilterStrat;
        }

        /**
         * @brief Set the Proximity Filter strategy of the ProximityCalculator.
         * @param proxStrategy [in] the new ProximityFilterStrategy. 
         * The strategy is not copied so changes to the strategy will affect the calculator
         */
        void setProximityFilterStrategy (rw::core::Ptr< ProximityFilterStrategy > proxStrategy)
        {
            _proxFilterStrat = proxStrategy;
        }

        /**
         * @brief Set a new strategy. OBS. models are stored in the strategy, so make sure that the
         * new strategy includes all nessesary models
         * @param strategy [in] the new strategy
         */
        void setStrategy (rw::core::Ptr< Strategy > strategy);

        /**
         * @brief Get the ProximityStrategy.
         * @return the strategy if set, otherwise NULL.
         */
        rw::core::Ptr< Strategy > getStrategy () const { return _strategy; }

        /**
         * @brief Add Geometry associated to \b frame
         *
         * The current shape of the geometry is copied, hence later changes to \b geometry has no
         * effect
         *
         * @param frame [in] Frame to associate geometry to
         * @param geometry [in] Geometry to add
         * @return true if succesful, otherwise false
         */
        bool addGeometry (rw::core::Ptr< rw::kinematics::Frame > frame,
                          const rw::core::Ptr< rw::geometry::Geometry >& geometry);

        /**
         * @brief Removes geometry from ProximityCalculator
         *
         * The id of the geometry is used to match the proximity model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometry [in] Geometry with the id to be removed
         */
        void removeGeometry (rw::core::Ptr< rw::kinematics::Frame > frame,
                             const rw::core::Ptr< rw::geometry::Geometry >& geometry);

        /**
         * @brief Removes geometry from ProximityCalculator
         *
         * The \b geometryId is used to match the proximity model to the geometry.
         *
         * @param frame [in] The frame which has the geometry associated
         * @param geometryId [in] Id of geometry to be removed
         */
        void removeGeometry (rw::core::Ptr< rw::kinematics::Frame > frame,
                             const std::string geometryId);

        //! @brief Adds rule specifying inclusion/exclusion of frame pairs in Proximity calculation
        void addRule (const rw::proximity::ProximitySetupRule& rule);

        //! @brief Removes rule specifying inclusion/exclusion of frame pairs in Proximity
        //! calculation
        void removeRule (const rw::proximity::ProximitySetupRule& rule);

        /**
         * @brief Get the computation time used in the inCollision functions.
         * @return the total computation time.
         */
        double getComputationTime () const { return _timer.getTime (); }

        /**
         * @brief Get the number of times the inCollision functions have been called.
         * @return number of calls to inCollision functions.
         */
        size_t getNoOfCalls () const { return _numberOfCalls; }

        /**
         * @brief Reset the counter for inCollision invocations and the computation timer.
         */
        void resetComputationTimeAndCount ()
        {
            _timer.resetAndPause ();
            _numberOfCalls = 0;
        }

        /**
         * @brief return the ids of all the geometries of this frames.
         */
        std::vector< std::string > getGeometryIDs (rw::core::Ptr< rw::kinematics::Frame > frame);

        /**
         * @brief Returns whether frame has an associated geometry with \b geometryId.
         * @param frame [in] Frame in question
         * @param geometryId [in] Id of the geometry
         */
        bool hasGeometry (rw::core::Ptr< rw::kinematics::Frame > frame,
                          const std::string& geometryId);

        /**
         * @brief Get the geometry from its ID
         * @param ID [in] the ID of the geometry
         * @return Pointer to the geometry
         */
        rw::core::Ptr< rw::geometry::Geometry >
        getGeometry (rw::core::Ptr< rw::kinematics::Frame > frame, const std::string& geometryId);

        /**
         * @brief static function to make a new ProximityCalculator
         *
         * Construct proximity calculator for a WorkCell with an associated
         * proximity strategy.
         *
         * The ProximityCalculator extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * The ProximityCalculator is initialized with the \b strategy .
         * Notice that the ProximityCalculator will create and store models inside the \b strategy .
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the ProximityStrategy to use
         */
        template< class R >
        static rw::core::Ptr< ProximityCalculator< R > >
        make (rw::core::Ptr< rw::models::WorkCell > workcell, rw::core::Ptr< R > strategy)
        {
            return rw::core::ownedPtr (new ProximityCalculator< R > (workcell, strategy));
        }

      private:
        rw::core::Ptr< ProximityFilterStrategy > _proxFilterStrat;
        rw::core::Ptr< CollisionSetup > _setup;
        rw::core::Ptr< Strategy > _strategy;
        rw::core::Ptr< Strategy > _thresholdStrategy;
        std::vector< std::pair< rw::core::Ptr< rw::kinematics::Frame >,
                                rw::core::Ptr< rw::kinematics::Frame > > >
            _distancePairs;
        rw::kinematics::State _state;
        rw::core::Ptr< rw::kinematics::Frame > _root;
        rw::kinematics::FrameMap< rw::core::Ptr< ProximityModel > > _frameToModels;

        rw::common::Timer _timer;
        size_t _numberOfCalls;

        void initGeom (rw::core::Ptr< rw::models::WorkCell > wc);
        void initDistPairs (const rw::kinematics::State& state);

#if __cplusplus < 201103L
        ProximityCalculator (const ProximityCalculator&);
        ProximityCalculator& operator= (const ProximityCalculator&);
#endif
    };

    #if defined(SWIG)
        SWIG_DECLARE_TEMPLATE(ProximityCalculatorCollision,rw::proximity::ProximityCalculator<rw::proximity::CollisionStrategy>);
        SWIG_DECLARE_TEMPLATE(ProximityCalculatorDistance,rw::proximity::ProximityCalculator<rw::proximity::DistanceStrategy>);
        SWIG_DECLARE_TEMPLATE(DistanceMultiCalculator,rw::proximity::ProximityCalculator<rw::proximity::DistanceMultiStrategy>);
    #endif 

    /*@}*/
}}    // namespace rw::proximity

#endif