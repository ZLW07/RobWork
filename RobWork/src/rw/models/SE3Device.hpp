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

#ifndef RW_MODELS_CARTESIAN6DOFDEVICE_HPP
#define RW_MODELS_CARTESIAN6DOFDEVICE_HPP

/**
 * @file SE3Device.hpp
 */
#if !defined(SWIG)
#include "Device.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#endif 

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Cartesian 6-Dof device
     *
     * The SE3Device is a 6-dof device with 6 independent inputs that
     * enables the device to place its end-effector anywhere in the workspace.
     *
     * The @f$ \mathbf{q}\in \mathbb{R}^6 @f$ input vector maps directly to the
     * end-effector pose @f$ \robabx{b}{e}{\mathbf{x}} @f$, thus:
     *
     * @f[ \robabx{b}{e}{\mathbf{x}} =
     * \left[
     * \begin{array}{c}
     * x\\
     * y\\
     * z\\
     * \theta k_x\\
     * \theta k_y\\
     * \theta k_z
     * \end{array}
     * \right] =
     * \left[
     * \begin{array}{c}
     * q_1\\
     * q_2\\
     * q_3\\
     * q_4\\
     * q_5\\
     * q_6
     * \end{array}
     * \right] =
     * \mathbf{q} @f]
     *
     * It is easily seen that the jacobian @f$
     * {^b_6}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \frac{\partial
     * ^b\mathbf{x}_6}{\partial \mathbf{q}} @f$ equals the @f$ 6\times 6 @f$
     * identity matrix @f$ \mathbf{I}^{6\times 6} @f$
     *
     * The device can be seen as a "perfect" robot, it has no singularities
     * anywhere in the task space, no kinematic or dynamic limits (it can
     * instantaneous move anywhere at any time). The device is interesting in
     * simulations where performance and stability of closed-loop control
     * systems (for instance visual-servoing systems) must be evaluated - if a
     * closed-loop control system does not perform well with a "perfect" robot
     * it will probably not perform well with a real robot either.
     */
    class SE3Device : public Device
    {
      public:
        /**
         * @brief Constructor
         *
         * @param name [in] device name
         * @param base documentation missing !
         * @param mframe documentation missing !
         */
        SE3Device (const std::string& name, rw::kinematics::Frame* base,
                   rw::kinematics::MovableFrame* mframe);

        virtual ~SE3Device () {}

        /**
         * @copydoc Device::setQ
         *
         * @pre q.size() == 6
         */
        void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         *
         * Since the SE3Device robot is unconstrained and can move anywhere
         * whithin the taskspace each of the 6 input's are unbounded (@f$
         * [-\inf, \inf] @f$) in practice the inputs are limited to the
         * numerical limits of the real datatype, thus this method returns the
         * range ([DBL_MIN, DBL_MAX]) for each of the 6 inputs
         */
        std::pair< rw::math::Q, rw::math::Q > getBounds () const;

        /**
         * @brief get base of the device
         * @return base Frame
         */
        rw::kinematics::Frame* getBase () { return _base; }

        /**
         * @brief get base of the device
         * @return base Frame
         */
        const rw::kinematics::Frame* getBase () const { return _base; }

        /**
         * @brief get end of the device
         * @return end Frame
         */
        rw::kinematics::Frame* getEnd () { return _mframe; }

        /**
         * @brief get end of the device
         * @return end Frame
         */
        const rw::kinematics::Frame* getEnd () const { return _mframe; }
#if ! defined(SWIGJAVA)
        /**
         * @brief Calculates the jacobian matrix of the end-effector described
         * in the robot base frame @f$ ^b_e\mathbf{J}_{\mathbf{q}}(\mathbf{q})
         * @f$
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^b_e}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * Where:
         *
         * \f[
         *  {^b_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \mathbf{I}^{6\times 6} =
         *  \left[
         *    \begin{array}{cccccc}
         *    1 & 0 & 0 & 0 & 0 & 0\\
         *    0 & 1 & 0 & 0 & 0 & 0\\
         *    0 & 0 & 1 & 0 & 0 & 0\\
         *    0 & 0 & 0 & 1 & 0 & 0\\
         *    0 & 0 & 0 & 0 & 1 & 0\\
         *    0 & 0 & 0 & 0 & 0 & 1\\
         *    \end{array}
         *  \right]
         * \f]
         *
         */

         #endif 
        rw::math::Jacobian baseJend (const rw::kinematics::State& state) const;

       rw::core::Ptr< rw::models::JacobianCalculator > baseJCframes (const std::vector< rw::kinematics::Frame* >& frames,
                                              const rw::kinematics::State& state) const
        {
            return NULL;
        }
        /**
         * @copydoc Device::getDOF
         *
         * This method always returns the value 6
         */
        size_t getDOF () const { return 6; }

        /**
         * @brief set outer bound of the device
         * @param bounds [in] the minimum Q and the maximum Q
         */
        virtual void setBounds (const QBox& bounds);

        /**
         * @brief get the Joint velocity limit
         * @return the velocity limit as Q
         */
        virtual rw::math::Q getVelocityLimits () const;

        /**
         * @brief set the Joint velocity limit
         * @param vellimits [in] the velocity limit as Q
         */
        virtual void setVelocityLimits (const rw::math::Q& vellimits);

        /**
         * @brief get the Joint Acceleration limit
         * @return the Acceleration limit as Q
         */
        rw::math::Q getAccelerationLimits () const;

        /**
         * @brief set the Joint Acceleration limit
         * @param acclimit [in] the acceleration limit as Q
         */
        void setAccelerationLimits (const rw::math::Q& acclimits);

      private:
        rw::kinematics::Frame* _base;
        rw::kinematics::MovableFrame* _mframe;
        rw::math::Q _vellimits, _acclimits;
    };

    /*@}*/
}}    // namespace rw::models

#endif    // end include guard
