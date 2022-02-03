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

#ifndef RW_MODELS_MOBILEDEVICE_HPP
#define RW_MODELS_MOBILEDEVICE_HPP

#if !defined(SWIG)
#include "Device.hpp"
#include "JointDevice.hpp"

#include <rw/math/Q.hpp>
#endif

namespace rw { namespace kinematics {
    class MovableFrame;
}}    // namespace rw::kinematics

namespace rw { namespace models {

    class RevoluteJoint;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Provides a differential controlled mobile device
     *
     * The MobileDevice class provides a differential controlled mobile device
     * with non-holonomic constraints. The \f$ x\f$ direction is defined as
     * straight forward and \f$ z\f$ vertically up. The wheels are assumed to be
     * positioned summetrically around the base and have \f$ 0\f$ \f$ x\f$
     * component.
     *
     * When using setQ it takes 2 parameters, which corresponds to the distances
     * travelled by the wheels. Based on this input and the current pose of the
     * device it calcualtes a new pose as.
     */
    class MobileDevice : public Device
    {
      public:
        /**
         * @brief Constructs a mobile device
         * @param base [in] the base of the device
         * @param wheel1 [in] the left wheel
         * @param wheel2 [in] the right wheel
         * @param state [in] the state of the device
         * @param name [in] name of device
         */
        MobileDevice (rw::kinematics::MovableFrame* base, RevoluteJoint* wheel1,
                      RevoluteJoint* wheel2, rw::kinematics::State& state, const std::string& name);

        /**
         * @brief Destructor
         */
        virtual ~MobileDevice ();

        /**
         * @brief Sets the position and orientation of the base
         *
         * This operation moves the base of the robot, without considering
         * the non-holonomic constraints of the device
         * @param transform [in] new base transform
         * @param state [in] state to write change to
         */
        void setDevicePose (const rw::math::Transform3D<>& transform, rw::kinematics::State& state);

        /**
         * @copydoc Device::setQ
         */
        virtual void setQ (const rw::math::Q& q, rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        virtual rw::math::Q getQ (const rw::kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         */
        virtual std::pair< rw::math::Q, rw::math::Q > getBounds () const;

        /**
         * @copydoc Device::setBounds
         */
        virtual void setBounds (const std::pair< rw::math::Q, rw::math::Q >& bounds);

        /**
         * @copydoc Device::getVelocityLimits
         */
        virtual rw::math::Q getVelocityLimits () const;

        /**
         * @copydoc Device::setVelocityLimits
         */
        virtual void setVelocityLimits (const rw::math::Q& vellimits);

        /**
         * @copydoc Device::getAccelerationLimits
         */
        virtual rw::math::Q getAccelerationLimits () const;

        /**
         * @copydoc Device::setAccelerationLimits
         */
        virtual void setAccelerationLimits (const rw::math::Q& acclimits);

        /**
         * @copydoc Device::getDOF
         */
        virtual size_t getDOF () const;

        /**
         * @copydoc Device::getBase()
         */
        virtual rw::kinematics::Frame* getBase ();

        /**
         * @copydoc Device::getBase() const
         */
        virtual const rw::kinematics::Frame* getBase () const;

        /**
         * @copydoc Device::getEnd()
         */
        virtual rw::kinematics::Frame* getEnd ();

        /**
         * @copydoc Device::getEnd() const
         */
        virtual const rw::kinematics::Frame* getEnd () const;

        /**
         * @copydoc Device::baseJend
         */
        virtual rw::math::Jacobian baseJend (const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJframe
           Not implemented.
        */
        virtual rw::math::Jacobian baseJframe (const rw::kinematics::Frame* frame,
                                               const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJframes
           Not implemented.
        */
        virtual rw::math::Jacobian baseJframes (const std::vector< rw::kinematics::Frame* >& frames,
                                                const rw::kinematics::State& state) const;

        /**
           @copydoc Device::baseJCframes
           Not implemented.
        */
        virtual rw::core::Ptr< rw::models::JacobianCalculator >
        baseJCframes (const std::vector< rw::kinematics::Frame* >& frames,
                      const rw::kinematics::State& state) const;

      private:
        rw::kinematics::MovableFrame* _base;
        std::vector< rw::kinematics::Frame* > _axillaryFrames;
        double _width;

        std::pair< rw::math::Q, rw::math::Q > _posLimits;
        rw::math::Q _velLimits;
        rw::math::Q _accLimits;

        // TODO: Implement without using JointDevice. This is an ugly hack
        rw::models::JointDevice _jointDevice;
        // BasicDevice _basicDevice;
    };

    /*@}*/

}}    // namespace rw::models

#endif    // end include guard
