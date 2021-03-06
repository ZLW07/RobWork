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

#include "Rotation3D.hpp"

#include "Math.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Rotation3DVector.hpp>

using namespace rw::common;
using namespace rw::math;

template< class T >
Rotation3D< T >::Rotation3D (const Rotation3DVector< T >& rotVec) :
    Rotation3D (rotVec.toRotation3D ())
{}

template< class T > const Rotation3D< T >& Rotation3D< T >::identity ()
{
    static Rotation3D id (1, 0, 0, 0, 1, 0, 0, 0, 1);
    return id;
}

template< class T > bool Rotation3D< T >::isProperRotation () const
{
    return LinearAlgebra::isSO (e ());
}

template< class T > bool Rotation3D< T >::isProperRotation (T precision) const
{
    return LinearAlgebra::isSO (e (), precision);
}

template< class T > void Rotation3D< T >::normalize ()
{
    T eps00, eps01, eps02, eps11, eps12, eps22, prod0, prod1, prod2, prod;
    prod0 = _m (0, 0) * _m (0, 0) + _m (1, 0) * _m (1, 0) + _m (2, 0) * _m (2, 0);
    eps00 = ((T) 1.0 - prod0) / prod0;
    prod1 = _m (0, 1) * _m (0, 1) + _m (1, 1) * _m (1, 1) + _m (2, 1) * _m (2, 1);
    eps11 = ((T) 1.0 - prod1) / prod1;
    prod2 = _m (0, 2) * _m (0, 2) + _m (1, 2) * _m (1, 2) + _m (2, 2) * _m (2, 2);
    eps22 = ((T) 1.0 - prod2) / prod2;
    prod  = _m (0, 0) * _m (0, 1) + _m (1, 0) * _m (1, 1) + _m (2, 0) * _m (2, 1);
    eps01 = -prod / (prod0 + prod1);
    prod  = _m (0, 0) * _m (0, 2) + _m (1, 0) * _m (1, 2) + _m (2, 0) * _m (2, 2);
    eps02 = -prod / (prod0 + prod2);
    prod  = _m (0, 1) * _m (0, 2) + _m (1, 1) * _m (1, 2) + _m (2, 1) * _m (2, 2);
    eps12 = -prod / (prod1 + prod2);
    _m (0, 0) += eps00 * _m (0, 0) + eps01 * _m (0, 1) + eps02 * _m (0, 2);
    _m (1, 0) += eps00 * _m (1, 0) + eps01 * _m (1, 1) + eps02 * _m (1, 2);
    _m (2, 0) += eps00 * _m (2, 0) + eps01 * _m (2, 1) + eps02 * _m (2, 2);
    _m (0, 1) += eps01 * _m (0, 0) + eps11 * _m (0, 1) + eps12 * _m (0, 2);
    _m (1, 1) += eps01 * _m (1, 0) + eps11 * _m (1, 1) + eps12 * _m (1, 2);
    _m (2, 1) += eps01 * _m (2, 0) + eps11 * _m (2, 1) + eps12 * _m (2, 2);
    _m (0, 2) += eps02 * _m (0, 0) + eps12 * _m (0, 1) + eps22 * _m (0, 2);
    _m (1, 2) += eps02 * _m (1, 0) + eps12 * _m (1, 1) + eps22 * _m (1, 2);
    _m (2, 2) += eps02 * _m (2, 0) + eps12 * _m (2, 1) + eps22 * _m (2, 2);

    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            if (_m (i, j) > 1)
                _m (i, j) = 1;
            else if (_m (i, j) < -1)
                _m (i, j) = -1;
        }
    }
}

template< class T >
void Rotation3D< T >::multiply (const Rotation3D< T >& a, const Vector3D< T >& b,
                                Vector3D< T >& result)
{
    const T a00 = a (0, 0);
    const T a01 = a (0, 1);
    const T a02 = a (0, 2);

    const T a10 = a (1, 0);
    const T a11 = a (1, 1);
    const T a12 = a (1, 2);

    const T a20 = a (2, 0);
    const T a21 = a (2, 1);
    const T a22 = a (2, 2);

    const T b03 = b (0);
    const T b13 = b (1);
    const T b23 = b (2);

    result (0) = a00 * b03 + a01 * b13 + a02 * b23;
    result (1) = a10 * b03 + a11 * b13 + a12 * b23;
    result (2) = a20 * b03 + a21 * b13 + a22 * b23;
}

template< class T >
const Rotation3D< T > Rotation3D< T >::multiply (const Rotation3D< T >& a, const Rotation3D< T >& b)
{
    const T a00 = a (0, 0);
    const T a01 = a (0, 1);
    const T a02 = a (0, 2);

    const T a10 = a (1, 0);
    const T a11 = a (1, 1);
    const T a12 = a (1, 2);

    const T a20 = a (2, 0);
    const T a21 = a (2, 1);
    const T a22 = a (2, 2);

    const T b00 = b (0, 0);
    const T b01 = b (0, 1);
    const T b02 = b (0, 2);

    const T b10 = b (1, 0);
    const T b11 = b (1, 1);
    const T b12 = b (1, 2);

    const T b20 = b (2, 0);
    const T b21 = b (2, 1);
    const T b22 = b (2, 2);

    return Rotation3D< T > (a00 * b00 + a01 * b10 + a02 * b20,

                            a00 * b01 + a01 * b11 + a02 * b21,

                            a00 * b02 + a01 * b12 + a02 * b22,

                            a10 * b00 + a11 * b10 + a12 * b20,

                            a10 * b01 + a11 * b11 + a12 * b21,

                            a10 * b02 + a11 * b12 + a12 * b22,

                            a20 * b00 + a21 * b10 + a22 * b20,

                            a20 * b01 + a21 * b11 + a22 * b21,

                            a20 * b02 + a21 * b12 + a22 * b22);
}

template< class T >
const Vector3D< T > Rotation3D< T >::multiply (const Rotation3D< T >& a, const Vector3D< T >& b)
{
    const T a00 = a (0, 0);
    const T a01 = a (0, 1);
    const T a02 = a (0, 2);

    const T a10 = a (1, 0);
    const T a11 = a (1, 1);
    const T a12 = a (1, 2);

    const T a20 = a (2, 0);
    const T a21 = a (2, 1);
    const T a22 = a (2, 2);

    const T b03 = b (0);
    const T b13 = b (1);
    const T b23 = b (2);

    return Vector3D< T > (a00 * b03 + a01 * b13 + a02 * b23,
                          a10 * b03 + a11 * b13 + a12 * b23,
                          a20 * b03 + a21 * b13 + a22 * b23);
}

template< class T >
void Rotation3D< T >::multiply (const Rotation3D< T >& a, const Rotation3D< T >& b,
                                Rotation3D< T >& result)
{
    const T a00 = a (0, 0);
    const T a01 = a (0, 1);
    const T a02 = a (0, 2);

    const T a10 = a (1, 0);
    const T a11 = a (1, 1);
    const T a12 = a (1, 2);

    const T a20 = a (2, 0);
    const T a21 = a (2, 1);
    const T a22 = a (2, 2);

    const T b00 = b (0, 0);
    const T b01 = b (0, 1);
    const T b02 = b (0, 2);

    const T b10 = b (1, 0);
    const T b11 = b (1, 1);
    const T b12 = b (1, 2);

    const T b20 = b (2, 0);
    const T b21 = b (2, 1);
    const T b22 = b (2, 2);

    result (0, 0) = a00 * b00 + a01 * b10 + a02 * b20;

    result (0, 1) = a00 * b01 + a01 * b11 + a02 * b21;

    result (0, 2) = a00 * b02 + a01 * b12 + a02 * b22;

    result (1, 0) = a10 * b00 + a11 * b10 + a12 * b20;

    result (1, 1) = a10 * b01 + a11 * b11 + a12 * b21;

    result (1, 2) = a10 * b02 + a11 * b12 + a12 * b22;

    result (2, 0) = a20 * b00 + a21 * b10 + a22 * b20;

    result (2, 1) = a20 * b01 + a21 * b11 + a22 * b21;

    result (2, 2) = a20 * b02 + a21 * b12 + a22 * b22;
}

template< class T > Rotation3D< T > Rotation3D< T >::inverse (bool copy)
{
    if (copy) {
        return Rotation3D< T > (_m (0, 0),
                                _m (1, 0),
                                _m (2, 0),
                                _m (0, 1),
                                _m (1, 1),
                                _m (2, 1),
                                _m (0, 2),
                                _m (1, 2),
                                _m (2, 2));
    }
    T tmpVal  = _m (0, 1);
    _m (0, 1) = _m (1, 0);
    _m (1, 0) = tmpVal;

    tmpVal    = _m (0, 2);
    _m (0, 2) = _m (2, 0);
    _m (2, 0) = tmpVal;

    tmpVal    = _m (1, 2);
    _m (1, 2) = _m (2, 1);
    _m (2, 1) = tmpVal;

    return *this;
}

template< class T > Rotation3D< T > Rotation3D< T >::inverse (bool copy) const
{
    if (!copy) {
        RW_THROW ("You can't modefy a constant object");
    }
    return Rotation3D< T > (_m (0, 0),
                            _m (1, 0),
                            _m (2, 0),
                            _m (0, 1),
                            _m (1, 1),
                            _m (2, 1),
                            _m (0, 2),
                            _m (1, 2),
                            _m (2, 2));
}

// Explicit template specifications.
template class rw::math::Rotation3D< double >;
template class rw::math::Rotation3D< float >;

namespace rw { namespace common { namespace serialization {

    template< class T >
    void writeImpl (const Rotation3D< T >& tmp, OutputArchive& oar, const std::string& id)
    {
        std::vector< double > data = Math::toStdVector (tmp, 3, 3);
        oar.write (data, id);
    }

    template< class T >
    void readImpl (Rotation3D< T >& tmp, InputArchive& iar, const std::string& id)
    {
        std::vector< T > data;
        iar.read (data, id);
        Math::fromStdVectorToMat (data, tmp, 3, 3);
    }

    // we need these to explicitly instantiate these functions
    template<>
    void write (const Rotation3D< double >& tmp, OutputArchive& oar, const std::string& id)
    {
        writeImpl (tmp, oar, id);
    }
    template<>
    void write (const Rotation3D< float >& tmp, OutputArchive& oar, const std::string& id)
    {
        writeImpl (tmp, oar, id);
    }
    template<> void read (Rotation3D< double >& tmp, InputArchive& iar, const std::string& id)
    {
        readImpl (tmp, iar, id);
    }
    template<> void read (Rotation3D< float >& tmp, InputArchive& iar, const std::string& id)
    {
        readImpl (tmp, iar, id);
    }

}}}    // namespace rw::common::serialization
