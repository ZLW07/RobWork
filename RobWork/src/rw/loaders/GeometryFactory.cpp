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

#include "GeometryFactory.hpp"

#include "Model3DFactory.hpp"
#include "model3d/STLFile.hpp"

#include <rw/core/Extension.hpp>
#include <rw/core/IOUtil.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Cone.hpp>
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/geometry/Pyramid.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Tube.hpp>

#include <string>
#include <sys/stat.h>

using namespace rw::loaders;

using namespace rw::core;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;

using namespace std;

namespace {
const std::string extensionsArray[] = {
    ".STL", ".STLA", ".STLB", ".PCD", ".TRI", ".DAE", ".AC", ".AC3D", ".3DS", ".OBJ"};

const int extensionCount = sizeof (extensionsArray) / sizeof (extensionsArray[0]);

const std::vector< std::string > extensions (extensionsArray, extensionsArray + extensionCount);

std::string getLastModifiedStr (const std::string& file)
{
    struct stat status;
    stat (file.c_str (), &status);
    // std::cout << "LAST MODIFIED DATE: " << status.st_mtime << std::endl;
    std::stringstream sstr;
    sstr << status.st_mtime;
    return sstr.str ();
}

Geometry::Ptr constructBox (std::stringstream& sstr)
{
    float x, y, z;
    sstr >> x >> y >> z;
    return ownedPtr (new Geometry (ownedPtr (new Box (x, y, z))));
}

Geometry::Ptr constructCylinder (std::stringstream& sstr)
{
    float radius, height;
    int divisions;
    if (sstr >> radius >> height >> divisions) {
        if (divisions < 0)
            RW_THROW ("Negative discretization level " << divisions);

        return ownedPtr (new Geometry (ownedPtr (new Cylinder (radius, height, divisions))));
    }
    else {
        RW_THROW ("Could not read (radius, height, divisions).");
        return NULL;
    }
}

Geometry::Ptr constructTube (std::stringstream& sstr)
{
    float radius, thickness, height;
    int divisions;
    if (sstr >> radius >> thickness >> height >> divisions) {
        if (divisions < 0)
            RW_THROW ("Negative discretization level " << divisions);
        return ownedPtr (new Geometry (ownedPtr (new Tube (radius, thickness, height, divisions))));
    }
    else {
        RW_THROW ("Could not read (radius, thickness, height, divisions).");
        return NULL;
    }
}

Geometry::Ptr constructCustom (std::stringstream& sstr)
{
    /*
     * Extract custom geometry parameters.
     */
    string geometryType, parameterString;

    if (sstr >> geometryType && getline (sstr, parameterString)) {
        pair< bool, vector< double > > parameters =
            StringUtil::toDoubles (StringUtil::words (parameterString));
        Q q (parameters.second);

        /*
         * Look up the geometry type in extensions.
         */
        GeometryFactory geoFactory;
        vector< Extension::Ptr > extensions = geoFactory.getExtensions ();

        for (Extension::Ptr& extension : extensions) {
            if (extension->getProperties ().get ("type", extension->getName ()) == geometryType) {
                GeometryData::Ptr geomData = extension->getObject ().cast< GeometryData > ();

                if (q.size () > 0) {
                    const Primitive::Ptr primitive = geomData.cast< Primitive > ();
                    if (!primitive.isNull ()) {
                        primitive->setParameters (q);
                    }
                }

                return ownedPtr (new Geometry (geomData));
            }
        }
    }
    else {
        RW_THROW ("Could not read (type, parameters).");
        return NULL;
    }

    return NULL;
}

Geometry::Ptr constructSphere (std::stringstream& sstr)
{
    float radius;
    int divisions;
    if (sstr >> radius >> divisions) {
        return ownedPtr (new Geometry (ownedPtr (new Sphere (radius, divisions))));
    }
    else {
        RW_THROW ("Could not read (radius).");
        return NULL;
    }
    return NULL;
}

Geometry::Ptr constructCone (std::stringstream& sstr)
{
    float height, radiusTop;
    int divisions;
    if (sstr >> radiusTop >> height >> divisions) {
        return ownedPtr (new Geometry (ownedPtr (new Cone (height, radiusTop, 0, divisions))));
    }
    else {
        RW_THROW ("Could not read (radius, height).");
        return NULL;
    }
    return NULL;
}

Geometry::Ptr constructLine (std::stringstream& sstr)
{
    RW_THROW ("Could not read (radius, height, divisions).");
    return NULL;
}

Geometry::Ptr constructPyramid (std::stringstream& sstr)
{
    float dx, dy, height;
    if (sstr >> dx >> dy >> height) {
        return ownedPtr (new Geometry (ownedPtr (new Pyramid (dx, dy, height))));
    }
    else {
        RW_THROW ("Could not read (dx, dy, height).");
        return NULL;
    }
    return NULL;
}

Geometry::Ptr constructPlane (std::stringstream& sstr)
{
    return ownedPtr (new Geometry (ownedPtr (new Plane (Vector3D<>::z (), 0))));
}
}    // namespace

Geometry::Ptr GeometryFactory::load (const std::string& raw_filename, bool useCache)
{
    return getGeometry (raw_filename, useCache);
}

Geometry::Ptr GeometryFactory::getGeometry (const std::string& raw_filename, bool useCache)
{
    if (raw_filename[0] != '#') {
        std::string filename;
        try {
            filename = IOUtil::resolveFileName (raw_filename, extensions);
        }
        catch (...) {
            // try loading a model instead, and create geometry from this
        }
        std::string filetype = StringUtil::toUpper (StringUtil::getFileExtension (filename));

        // if the file does not exist then throw an exception
        if (filetype.empty ()) {
            RW_WARN ("No file type known for file " << StringUtil::quote (raw_filename)
                                                    << " that was resolved to file name "
                                                    << filename << ": defaults to STL!");
            filetype = ".STL";
        }

        std::string moddate = getLastModifiedStr (filename);
        if (useCache && getCache ().isInCache (filename, moddate))
            return ownedPtr (new Geometry (getCache ().get (filename)));

        if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
            GeometryData::Ptr data = STLFile::load (filename);
            if (data == NULL)
                RW_THROW ("Reading of geometry failed!");
            getCache ().add (filename, data, moddate);
            return ownedPtr (new Geometry (getCache ().get (filename)));
        }
        else if (filetype == ".PCD") {
            GeometryData::Ptr data = PointCloud::loadPCD (filename);

            if (data == NULL)
                RW_THROW ("Reading of geometry failed!");
            getCache ().add (filename, data, moddate);
            return ownedPtr (new Geometry (getCache ().get (filename)));
        }
        else {
            rw::graphics::Model3D::Ptr model =
                rw::loaders::Model3DFactory::loadModel (filename, "");

            GeometryData::Ptr data = model->toGeometryData ();
            getCache ().add (filename, data, moddate);
            return ownedPtr (new Geometry (getCache ().get (filename)));
        }
    }

    std::stringstream sstr (raw_filename);
    std::string type;
    sstr >> type;

    if (type == "#Plane")
        return constructPlane (sstr);
    if (type == "#Box")
        return constructBox (sstr);
    if (type == "#Cylinder")
        return constructCylinder (sstr);
    if (type == "#Tube")
        return constructTube (sstr);
    if (type == "#Cone")
        return constructCone (sstr);
    if (type == "#Line")
        return constructLine (sstr);
    if (type == "#Sphere")
        return constructSphere (sstr);
    if (type == "#Pyramid")
        return constructPyramid (sstr);
    if (type == "#Custom")
        return constructCustom (sstr);
    else {
        RW_THROW ("Unable to construct geometry from string: \"" << raw_filename << "\"");
        // To avoid a compiler warning.
        return NULL;
    }

    RW_ASSERT (!"Impossible");
    return NULL;    // To avoid a compiler warning.
}

GeometryFactory::GeometryFactory () :
    ExtensionPoint< GeometryData > (
        "rw.loaders.GeometryFactory",
        "extension point for adding custom geometry primitives to the XML workcell format")
{}

GeometryFactory::Cache& GeometryFactory::getCache ()
{
    static Cache cache;
    return cache;
}

void GeometryFactory::clearGeometryCache ()
{
    getCache ().clear ();
}
