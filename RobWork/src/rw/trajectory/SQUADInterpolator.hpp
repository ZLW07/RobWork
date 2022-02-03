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

#ifndef RW_TRAJECTORY_RAMPINTERPOLATOR_HPP
#define RW_TRAJECTORY_RAMPINTERPOLATOR_HPP

/**
 * @file RampInterpolator.hpp
 */

#include "Interpolator.hpp"

#include <rw/core/macros.hpp>
#include <rw/math/Quaternion.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief
     */
    template< class T > class SQUADInterpolator : public Interpolator< rw::math::Quaternion< T > >
    {
      public:
        //! @brief smart pointer type to this class
        typedef typename rw::core::Ptr< SQUADInterpolator< rw::math::Quaternion< T > > > Ptr;

        SQUADInterpolator (rw::math::Quaternion< T > q0, rw::math::Quaternion< T > q1,
                           rw::math::Quaternion< T > s0, rw::math::Quaternion< T > s1,
                           double duration) :
            _duration (duration),
            _q0 (q0), _q1 (q1), _s0 (s0), _s1 (s1)
        {}

        rw::math::Quaternion< T > x (double t) const
        {
            double h = t / _duration;
            return Slerp (Slerp (_q0, _q1, h), Slerp (_s0, _s1, h), 2 * h * (1 - h));
        }

        rw::math::Quaternion< T > dx (double t) const
        {
            RW_THROW ("dx not implemented for SQUADInterpolation");
            return rw::math::Quaternion< T > ();
        }

        rw::math::Quaternion< T > ddx (double t) const
        {
            RW_THROW ("dx not implemented for SQUADInterpolation");
            return rw::math::Quaternion< T > ();
        }

        double duration () const { return _duration; }

      private:
        rw::math::Quaternion< T > Slerp (rw::math::Quaternion< T > q0, rw::math::Quaternion< T > q1,
                                         double h) const
        {
            return q0.slerp (q1, h);
        }
        double _duration;
        rw::math::Quaternion< T > _q0;
        rw::math::Quaternion< T > _q1;
        rw::math::Quaternion< T > _s0;
        rw::math::Quaternion< T > _s1;
    };

    /** @} */

}}    // namespace rw::trajectory

#endif    // end include guard
