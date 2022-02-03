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

#ifndef RW_MODELS_DEVICEJACOBIANCALCULATOR_HPP
#define RW_MODELS_DEVICEJACOBIANCALCULATOR_HPP

#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/models/JacobianCalculator.hpp>

#include <vector>
#endif 

namespace rw { namespace kinematics {
    class State;
    class Frame;
}}    // namespace rw::kinematics

namespace rw { namespace models {
    class Device;
    class Joint;

    /**
     * @brief Calculator for Jacobians of one or several Devices.
     *
     * Implements Jacobian calculations for one or several Devices.
     *
     * If more than one end-effector is given a "stacked" Jacobian is returned.
     *
     */
    class DeviceJacobianCalculator : public JacobianCalculator
    {
      public:
        /**
         * @brief Constructs JacobianCalculator.
         *
         * The dimension of the jacobian wil be (tcps.size() * 6, device.getDOF()).
         *
         * @param devices [in] The device to calculate for
         * @param base [in] Reference base of the Jacobian. Does not have to be the same as the base
         * of the device
         * @param tcps [in] List of tool end-effectors for which to calculate the Jacobian.
         * @param state [in] State giving how frame are connected
         */
        DeviceJacobianCalculator (std::vector< rw::core::Ptr< rw::models::Device > > devices,
                                  const rw::kinematics::Frame* base,
                                  const std::vector< rw::kinematics::Frame* >& tcps,
                                  const rw::kinematics::State& state);

        /**
         * @brief Destructor
         */
        virtual ~DeviceJacobianCalculator ();

        /**
         * @copydoc JacobianCalculator::get(const rw::kinematics::State&) const
         */
        virtual rw::math::Jacobian get (const rw::kinematics::State& state) const;

      private:
        typedef std::vector< std::pair< const rw::models::Joint*, size_t > > JacobianSetup;

        std::vector< JacobianSetup > _jacobianSetups;
        std::vector< rw::core::Ptr< rw::models::Device > > _devices;
        const rw::kinematics::Frame* _base;
        std::vector< rw::kinematics::Frame* > _tcps;
        size_t _dof;
    };

}}    // namespace rw::models

#endif /* JOINTDEVICEJACOBIANCALCULATOR_HPP_ */
