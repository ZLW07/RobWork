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

#ifndef RW_MODELS_PARALLELLEG_HPP
#define RW_MODELS_PARALLELLEG_HPP

/**
 * @file ParallelLeg.hpp
 *
 * \copydoc rw::models::ParallelLeg
 */
#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/math/Transform3D.hpp>

#include <vector>
#endif 

namespace rw {
namespace math {
    class Jacobian;
    class Q;
}    // namespace math
namespace models {
    class Joint;
}
namespace kinematics {
    class State;
    class Frame;
}    // namespace kinematics
}    // namespace rw

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Class representing a single leg in a ParallelDevice
     */
    class ParallelLeg
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< ParallelLeg > Ptr;

        /**
         * @brief Constructs leg from frames
         * @param frames [in] list of Frame's
         */
        ParallelLeg (std::vector< rw::kinematics::Frame* > frames);

        /**
         * @brief Destructor
         */
        virtual ~ParallelLeg ();

        /**
         * @brief Returns the base to end Jacobian
         * @param state [in] State for which to calculate the Jacobian
         * @return the Jacobian
         */
        const rw::math::Jacobian& baseJend (const rw::kinematics::State& state);

        /**
         * @brief Returns the Jacobian of \b frame relative to base frame.
         * @param frame [in] the frame to find Jacobian for.
         * @param state [in] State for which to calculate the Jacobian
         * @return the Jacobian
         */
        rw::math::Jacobian baseJframe (const rw::kinematics::Frame* frame,
                                       const rw::kinematics::State& state) const;

        /**
         * @brief Returns the base to end transformation
         * @param state [in] State for which to calculate the transform
         * @return the transform
         */
        rw::math::Transform3D< double > baseTend (const rw::kinematics::State& state) const;

        /**
         * @brief Returns the transformation of a \b frame relative to the base.
         * @param frame [in] the frame to find transformation to.
         * @param state [in] State for which to calculate the transform
         * @return the transform
         */
        rw::math::Transform3D< double > baseTframe (const rw::kinematics::Frame* frame,
                                                    const rw::kinematics::State& state) const;

        /**
         * @brief Returns the kinematic chain of the leg
         * @return list of frames
         */
        const std::vector< rw::kinematics::Frame* >& getKinematicChain () const;

        /**
         * @brief the base of the leg
         * @return the frame
         */
        rw::kinematics::Frame* getBase ();

        /**
         * @brief the end of the leg
         * @return the frame
         */
        rw::kinematics::Frame* getEnd ();

        /**
         * @brief Number of active joints
         * @return number of active joints
         */
        size_t nrOfActiveJoints ();

        /**
         * @brief Number of passive joints
         * @return number of passive joints
         */
        size_t nrOfPassiveJoints ();

        /**
         * @brief Number of joints (both active and passive)
         * @return number of joints
         */
        size_t nrOfJoints () { return _actuatedJoints.size () + _unactuatedJoints.size (); }

        /**
         * @brief Returns list of the actuated (active) joints
         * @return list of joints
         */
        const std::vector< models::Joint* >& getActuatedJoints () { return _actuatedJoints; }

        /**
         * @brief Returns list of unactuated (passive) joints
         * @return list of joints
         */
        const std::vector< models::Joint* >& getUnactuatedJoints () { return _unactuatedJoints; }

        /**
         * @brief Get the total degrees of freedom (includes both active and passive joints).
         * @return the total degrees of freedom.
         */
        std::size_t getJointDOFs () const;

        /**
         * @brief Get configuration of the leg.
         * @param state [in] the state with the configuration values.
         * @return the configuration.
         */
        rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @brief Sets q for the leg in the state
         * @param q [in] q to set
         * @param state [out] the State to modify
         */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

      private:
        std::vector< rw::kinematics::Frame* > _kinematicChain;
        std::vector< models::Joint* > _actuatedJoints;
        std::vector< models::Joint* > _unactuatedJoints;
        rw::math::Jacobian* _jacobian;
    };

    /*@}*/

}}    // namespace rw::models

#endif    // end include guard
