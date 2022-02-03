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

#ifndef RW_MODELS_PRISMATICUNIVERSALJOINT_HPP_
#define RW_MODELS_PRISMATICUNIVERSALJOINT_HPP_

/**
 * @file PrismaticUniversalJoint.hpp
 *
 * \copydoc rw::models::PrismaticUniversalJoint
 */
#if !defined(SWIG)
#include "Joint.hpp"
#endif

namespace rw { namespace models {
    //! @addtogroup models
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief A prismatic universal joint that allows rotations in two directions and translation
     * along the third.
     *
     * Rotation is allowed around the x and y axes. The xy-position is fixed, while the z-axis is
     * translational.
     *
     * This joint is equivalent to a universal joint followed by a translational joint.
     */
    class PrismaticUniversalJoint : public Joint
    {
      public:
        //! @brief Smart pointer type of PrismaticUniversalJoint
        typedef rw::core::Ptr< PrismaticUniversalJoint > Ptr;

        /**
         * @brief Construct a prismatic universal joint.
         * @param name [in] name of the joint.
         * @param transform [in] static transform of the joint.
         */
        PrismaticUniversalJoint (const std::string& name, const rw::math::Transform3D<>& transform);

        //! @brief Destructor.
        virtual ~PrismaticUniversalJoint ();

        // From Frame
        //! @brief Frame::doMultiplyTransform
        virtual void doMultiplyTransform (const rw::math::Transform3D<>& parent,
                                          const rw::kinematics::State& state,
                                          rw::math::Transform3D<>& result) const;

        //! @brief Frame::doGetTransform
        virtual rw::math::Transform3D<> doGetTransform (const rw::kinematics::State& state) const;

        // From Joint
        //! @copydoc rw::models::Joint::getJacobian
        virtual void getJacobian (std::size_t row, std::size_t col,
                                  const rw::math::Transform3D<>& joint,
                                  const rw::math::Transform3D<>& tcp,
                                  const rw::kinematics::State& state,
                                  rw::math::Jacobian& jacobian) const;

        //! @copydoc rw::models::Joint::getFixedTransform
        virtual rw::math::Transform3D<> getFixedTransform () const;

        //! @copydoc rw::models::Joint::setFixedTransform
        virtual void setFixedTransform (const rw::math::Transform3D<>& t3d);

        //! @copydoc rw::models::Joint::getJointTransform
        virtual rw::math::Transform3D<>
        getJointTransform (const rw::kinematics::State& state) const;

        //! @copydoc rw::models::Joint::setJointMapping
        virtual void setJointMapping (rw::math::Function1Diff<>::Ptr function);

        //! @copydoc rw::models::Joint::removeJointMapping
        virtual void removeJointMapping ();

      private:
        rw::math::Transform3D<> _T;
        rw::math::Function1Diff<>::Ptr _mapping;
    };
#if !defined(SWIG)
//! @}
#endif
}}    // namespace rw::models

#endif /* RW_MODELS_PRISMATICUNIVERSALJOINT_HPP_ */
