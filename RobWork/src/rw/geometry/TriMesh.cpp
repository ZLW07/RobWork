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

#include "TriMesh.hpp"

using namespace rw::geometry;
using namespace rw::math;

rw::core::Ptr< const TriMesh > TriMesh::getTriMesh (bool forceCopy) const
{
    if (forceCopy) {
        return clone ();
    }
    return rw::core::Ptr< const TriMesh > (this);
}

rw::core::Ptr< TriMesh > TriMesh::getTriMesh (bool forceCopy)
{
    if (forceCopy) {
        return clone ();
    }
    return TriMesh::Ptr (this);
}

double TriMesh::getVolume () const
{
    double volume = 0.0;

    // iterate over triangles
    for (size_t i = 0; i < size (); ++i) {
        Triangle< double > tri = getTriangle (i);

        Vector3D< double > v0 = tri.getVertex (0);
        Vector3D< double > v1 = tri.getVertex (1);
        Vector3D< double > v2 = tri.getVertex (2);

        double dv = dot (v0, cross (v1, v2));
        volume += dv;
    }

    return volume / 6.0;
}

void TriMesh::TriCenterIterator::inc ()
{
    using namespace rw::geometry;
    using namespace rw::math;
    using namespace rw::common;

    ++_first;
    if (_first != _end) {
        Triangle<> tri = _mesh.getTriangle (_first);
        if (_useAreaWeight) {
            double area = tri.calcArea ();
            _pos        = area * (tri.getVertex (0) + tri.getVertex (1) + tri.getVertex (2)) / 3.0;
        }
        else {
            _pos = (tri.getVertex (0) + tri.getVertex (1) + tri.getVertex (2)) / 3.0;
        }
    }
}