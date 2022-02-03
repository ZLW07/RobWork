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

#include "Quaternion.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Vector.hpp>

#include <math.h>

using namespace rw::common;
using namespace rw::math;

template< class T > Quaternion< T > Quaternion< T >::exp () const
{
    T a = getQw ();
    Vector3D< T > v (getQx (), getQy (), getQz ());
    T w = std::exp (a) * cos (v.norm2 ());

    v = (v / v.norm2 ()) * sin (v.norm2 ()) * T(std::exp (a));

    return Quaternion< T > (v[0], v[1], v[2], w);
}

template< class T > Quaternion< T > Quaternion< T >::inverse () const
{
    Quaternion< T > ret;

    ret = (*this) * (T(-1.0) / getLengthSquared ());
    ret[3] *= T(-1.0);
    return ret;
}

template< class T > Quaternion< T > Quaternion< T >::ln () const
{
    T a = acos (getQw ());
    T x = getQx () / sin (a);
    T y = getQy () / sin (a);
    T z = getQz () / sin (a);

    return Quaternion< T > (a * x, a * y, a * z, T(0));
}

template< class T > Quaternion< T > Quaternion< T >::pow (double power) const
{
    Quaternion< T > ret = *this;
    for (int i = 0; i < 4; i++) {
        ret[i] = std::pow (ret[i], power);
    }
    return ret;
}

template< class T > Quaternion< T > Quaternion< T >::elemDivide (const T& lhs) const
{
    Quaternion< T > ret;
    for (size_t i = 0; i < this->size (); i++) {
        ret[i] = (*this)[i] / lhs;
    }
    return ret;
}

template class rw::math::Quaternion< double >;
template class rw::math::Quaternion< float >;

template<>
void rw::common::serialization::write (const Quaternion< double >& tmp, OutputArchive& oar,
                                       const std::string& id)
{
    oar.write (Math::toStdVector (tmp, (int) tmp.size ()), id, "Quaternion");
}
template<>
void rw::common::serialization::read (Quaternion< double >& tmp, InputArchive& iar,
                                      const std::string& id)
{
    std::vector< double > arr;
    iar.read (arr, id, "Quaternion");
    Math::fromStdVector (arr, tmp);
}
template<>
void rw::common::serialization::write (const Quaternion< float >& tmp, OutputArchive& oar,
                                       const std::string& id)
{
    oar.write (Math::toStdVector (tmp, (int) tmp.size ()), id, "Quaternion");
}
template<>
void rw::common::serialization::read (Quaternion< float >& tmp, InputArchive& iar,
                                      const std::string& id)
{
    std::vector< float > arr;
    iar.read (arr, id, "Quaternion");
    Math::fromStdVector (arr, tmp);
}
