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

#include "Geometry.hpp"

#include "Box.hpp"
#include "Cone.hpp"
#include "Cylinder.hpp"
#include "IndexedTriMesh.hpp"
#include "Sphere.hpp"

#include <rw/math/Random.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::core;

namespace {

std::string makeName (GeometryData::GeometryType gtype)
{
    int ri = Random::ranI (0xFF, 0xFFFFFF);
    std::stringstream sstr;
    sstr << GeometryData::toString (gtype) << "_" << ri;
    return sstr.str ();
}

}    // namespace

Geometry::Geometry (GeometryData::Ptr data, double scale) :
    _refFrame (NULL), _data (data), _transform (rw::math::Transform3D<>::identity ()),
    _scale (scale), _name (makeName (data->getType ())), _filePath (""),
    _mask (Geometry::CollisionGroup), _colorRGB{0.8f, 0.8f, 0.8f}
{}

Geometry::Geometry (GeometryData::Ptr data, const std::string& name, double scale) :
    _refFrame (NULL), _data (data), _transform (rw::math::Transform3D<>::identity ()),
    _scale (scale), _name (name), _filePath (""),
    _mask (Geometry::CollisionGroup), _colorRGB{0.8f, 0.8f, 0.8f}
{}

Geometry::Geometry (GeometryData::Ptr data, const rw::math::Transform3D<>& t3d, double scale) :
    _refFrame (NULL), _data (data), _transform (t3d), _scale (scale),
    _name (makeName (data->getType ())), _filePath (""),
    _mask (Geometry::CollisionGroup), _colorRGB{0.8f, 0.8f, 0.8f}
{}

Geometry::~Geometry ()
{}

Geometry::Ptr Geometry::makeSphere (double radi)
{
    return ownedPtr (new Geometry (ownedPtr (new Sphere (radi))));
}

Geometry::Ptr Geometry::makeBox (double x, double y, double z)
{
    return ownedPtr (new Geometry (ownedPtr (new Box (x, y, z))));
}

Geometry::Ptr Geometry::makeCone (double height, double radiusTop, double radiusBot)
{
    return ownedPtr (new Geometry (ownedPtr (new Cone (height, radiusTop, radiusBot))));
}

Geometry::Ptr Geometry::makeCylinder (float radius, float height)
{
    return ownedPtr (new Geometry (ownedPtr (new Cylinder (radius, height))));
}

Geometry::Ptr Geometry::makeGrid (int dim_x, int dim_y, double size_x, double size_y,
                                  const rw::math::Vector3D<>& xdir,
                                  const rw::math::Vector3D<>& ydir)
{
    rw::geometry::IndexedTriMeshN0< float >::Ptr imesh =
        ownedPtr (new rw::geometry::IndexedTriMeshN0< float > ());

    rw::math::Vector3D< float > xdir_n = cast< float > (normalize (xdir));
    rw::math::Vector3D< float > ydir_n = cast< float > (normalize (ydir));

    for (int dy = 0; dy <= dim_y; dy++) {
        for (int dx = 0; dx <= dim_x; dx++) {
            imesh->getVertices ().push_back ((xdir_n * (float) (dx - dim_x / 2.0 * size_x) +
                                              ydir_n * (float) (dy - dim_y / 2.0 * size_y)));
        }
    }

    for (int dy = 0; dy <= dim_y - 1; dy++) {
        for (int dx = 0; dx <= dim_x - 1; dx++) {
            imesh->add (IndexedTriangle<> (
                dx + 1 + dy * dim_y, dx + dy * dim_y, dx + 1 + (dy + 1) * dim_y));
            imesh->add (IndexedTriangle<> (
                dx + (dy + 1) * dim_y, dx + 1 + (dy + 1) * dim_y, dx + 1 + dy * dim_y));
        }
    }

    return ownedPtr (new Geometry (imesh));
}

void Geometry::setColor (float red, float green, float blue)
{
    if (red > 1) {
        red = 1;
    }
    if (red < 0) {
        red = 0;
    }
    if (green > 1) {
        green = 1;
    }
    if (green < 0) {
        green = 0;
    }
    if (blue > 1) {
        blue = 1;
    }
    if (blue < 0) {
        blue = 0;
    }
    _colorRGB[0] = red;
    _colorRGB[1] = green;
    _colorRGB[2] = blue;
}