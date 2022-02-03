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

#include "CubicSplineInterpolator.hpp"

#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

// ########################## Function Constructor ##################################
namespace rw { namespace trajectory {

    template<>
    CubicSplineInterpolator< rw::math::Rotation3D< double > >::CubicSplineInterpolator (
        const rw::math::Rotation3D< double >& a, const rw::math::Rotation3D< double >& b,
        const rw::math::Rotation3D< double >& c, const rw::math::Rotation3D< double >& d,
        double duration)
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    CubicSplineInterpolator< rw::math::Rotation3D< float > >::CubicSplineInterpolator (
        const rw::math::Rotation3D< float >& a, const rw::math::Rotation3D< float >& b,
        const rw::math::Rotation3D< float >& c, const rw::math::Rotation3D< float >& d,
        double duration)
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    CubicSplineInterpolator< rw::math::Transform3D< double > >::CubicSplineInterpolator (
        const rw::math::Transform3D< double >& a, const rw::math::Transform3D< double >& b,
        const rw::math::Transform3D< double >& c, const rw::math::Transform3D< double >& d,
        double duration)
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    CubicSplineInterpolator< rw::math::Transform3D< float > >::CubicSplineInterpolator (
        const rw::math::Transform3D< float >& a, const rw::math::Transform3D< float >& b,
        const rw::math::Transform3D< float >& c, const rw::math::Transform3D< float >& d,
        double duration)
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    // ########################## Function X ##################################
    template<>
    rw::math::Quaternion< double >
    CubicSplineInterpolator< rw::math::Quaternion< double > >::x (double t) const
    {
        double tpow2                       = t * t;
        double tpow3                       = tpow2 * t;
        rw::math::Quaternion< double > rot = _a + _b * t + _c * tpow2 + _d * tpow3;
        rot.normalize ();
        return rot;
    }

    template<>
    rw::math::Quaternion< float >
    CubicSplineInterpolator< rw::math::Quaternion< float > >::x (double t) const
    {
        double tpow2                      = t * t;
        double tpow3                      = tpow2 * t;
        rw::math::Quaternion< float > rot = _a + _b * t + _c * tpow2 + _d * tpow3;
        rot.normalize ();
        return rot;
    }

    template<>
    rw::math::Transform3D< double >
    CubicSplineInterpolator< rw::math::Transform3D< double > >::x (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Transform3D< float >
    CubicSplineInterpolator< rw::math::Transform3D< float > >::x (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< double >
    CubicSplineInterpolator< rw::math::Rotation3D< double > >::x (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< float >
    CubicSplineInterpolator< rw::math::Rotation3D< float > >::x (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    // ########################## Function dX ##################################

    template<>
    rw::math::Transform3D< double >
    CubicSplineInterpolator< rw::math::Transform3D< double > >::dx (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Transform3D< float >
    CubicSplineInterpolator< rw::math::Transform3D< float > >::dx (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< double >
    CubicSplineInterpolator< rw::math::Rotation3D< double > >::dx (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< float >
    CubicSplineInterpolator< rw::math::Rotation3D< float > >::dx (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Quaternion< double >
    CubicSplineInterpolator< rw::math::Quaternion< double > >::dx (double t) const
    {
        RW_THROW ("Quaternion.dx is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Quaternion< float >
    CubicSplineInterpolator< rw::math::Quaternion< float > >::dx (double t) const
    {
        RW_THROW ("Quaternion.dx is not supported for CubicSplineInterpolator");
    }

    // ########################## Function ddX ##################################

    template<>
    rw::math::Transform3D< double >
    CubicSplineInterpolator< rw::math::Transform3D< double > >::ddx (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Transform3D< float >
    CubicSplineInterpolator< rw::math::Transform3D< float > >::ddx (double t) const
    {
        RW_THROW ("Transform3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< double >
    CubicSplineInterpolator< rw::math::Rotation3D< double > >::ddx (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Rotation3D< float >
    CubicSplineInterpolator< rw::math::Rotation3D< float > >::ddx (double t) const
    {
        RW_THROW ("Rotation3D is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Quaternion< double >
    CubicSplineInterpolator< rw::math::Quaternion< double > >::ddx (double t) const
    {
        RW_THROW ("Quaternion.ddx is not supported for CubicSplineInterpolator");
    }

    template<>
    rw::math::Quaternion< float >
    CubicSplineInterpolator< rw::math::Quaternion< float > >::ddx (double t) const
    {
        RW_THROW ("Quaternion.ddx is not supported for CubicSplineInterpolator");
    }

}}    // namespace rw::trajectory