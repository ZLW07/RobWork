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

#include "DOMPropertyMapLoader.hpp"

#include "DOMBasisTypes.hpp"
#include "DOMPathLoader.hpp"

#include <rw/core/DOMElem.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::core;
using namespace rw::loaders;
using namespace rw::trajectory;

DOMPropertyMapLoader::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        DOMBasisTypes::Initializer init1;
        DOMPropertyMapFormat::Initializer init2;
        done = true;
    }
}

const DOMPropertyMapLoader::Initializer DOMPropertyMapLoader::initializer;

DOMPropertyMapLoader::DOMPropertyMapLoader ()
{}

DOMPropertyMapLoader::~DOMPropertyMapLoader ()
{}

PropertyValueBase::Ptr DOMPropertyMapLoader::readPropertyValue (DOMElem::Ptr child)
{
    if (child->isName (DOMPropertyMapFormat::idPropertyMap ())) {
        return ownedPtr (new PropertyValue< PropertyMap > (
            DOMPropertyMapLoader::readProperties (child, true)));
    }
    else if (child->isName (DOMPropertyMapFormat::idPropertyValueList ())) {
        std::vector<PropertyValueBase::Ptr> list;
        for (DOMElem::Ptr val : child->getChildren ()) {
            list.push_back(readPropertyValue(val));
        }
        return ownedPtr (new PropertyValue< std::vector< PropertyValueBase::Ptr > > (list));
    }
    else if (child->isName (DOMBasisTypes::idString ())) {
        return ownedPtr (
            new PropertyValue< std::string > (DOMBasisTypes::readString (child)));
    }
    else if (child->isName (DOMBasisTypes::idStringList ())) {
        return ownedPtr (new PropertyValue< std::vector< std::string > > (
            DOMBasisTypes::readStringList (child)));
    }
    else if (child->isName (DOMBasisTypes::idIntList ())) {
        return ownedPtr (new PropertyValue< std::vector< int > > (
            DOMBasisTypes::readIntList (child)));
    }
    else if (child->isName (DOMBasisTypes::idDoubleList ())) {
        return ownedPtr (new PropertyValue< std::vector< double > > (
            DOMBasisTypes::readDoubleList (child)));
    }
    else if (child->isName (DOMBasisTypes::idDouble ())) {
        return ownedPtr (
            new PropertyValue< double > (DOMBasisTypes::readDouble (child)));
    }
    else if (child->isName (DOMBasisTypes::idFloat ())) {
        return ownedPtr (
            new PropertyValue< float > (DOMBasisTypes::readFloat (child)));
    }
    else if (child->isName (DOMBasisTypes::idInteger ())) {
        return ownedPtr (
            new PropertyValue< int > (DOMBasisTypes::readInt (child)));
    }
    else if (child->isName (DOMBasisTypes::idBoolean ())) {
        return ownedPtr (
            new PropertyValue< bool > (DOMBasisTypes::readBool (child)));
    }
    else if (child->isName (DOMBasisTypes::idVector3D ())) {
        return ownedPtr (new PropertyValue< Vector3D<> > (
            DOMBasisTypes::readVector3D (child)));
    }
    else if (child->isName (DOMBasisTypes::idVector2D ())) {
        return ownedPtr (new PropertyValue< Vector2D<> > (
            DOMBasisTypes::readVector2D (child)));
    }
    else if (child->isName (DOMBasisTypes::idQ ())) {
        return ownedPtr (new PropertyValue< Q > (DOMBasisTypes::readQ (child)));
    }
    else if (child->isName (DOMBasisTypes::idTransform3D ())) {
        return ownedPtr (new PropertyValue< Transform3D<> > (
            DOMBasisTypes::readTransform3D (child)));
    }
    else if (child->isName (DOMBasisTypes::idRotation3D ())) {
        return ownedPtr (new PropertyValue< Rotation3D<> > (
            DOMBasisTypes::readRotation3D (child)));
    }
    else if (child->isName (DOMBasisTypes::idEAA ())) {
        return ownedPtr (
            new PropertyValue< EAA<> > (DOMBasisTypes::readEAA (child)));
    }
    else if (child->isName (DOMBasisTypes::idRPY ())) {
        return ownedPtr (
            new PropertyValue< RPY<> > (DOMBasisTypes::readRPY (child)));
    }
    else if (child->isName (DOMBasisTypes::idQuaternion ())) {
        return ownedPtr (new PropertyValue< Quaternion<> > (
            DOMBasisTypes::readQuaternion (child)));
    }
    else if (child->isName (DOMBasisTypes::idRotation2D ())) {
        return ownedPtr (new PropertyValue< Rotation2D<> > (
            DOMBasisTypes::readRotation2D (child)));
    }
    else if (child->isName (DOMBasisTypes::idVelocityScrew6D ())) {
        return ownedPtr (new PropertyValue< VelocityScrew6D<> > (
            DOMBasisTypes::readVelocityScrew6D (child)));
    }
    else if (child->isName (DOMPathLoader::idQPath ())) {
        DOMPathLoader loader (child);
        return ownedPtr (new PropertyValue< QPath > (*loader.getQPath ()));
    }
    else if (child->isName (DOMPathLoader::idT3DPath ())) {
        DOMPathLoader loader (child);
        return ownedPtr (
            new PropertyValue< Transform3DPath > (*loader.getTransform3DPath ()));
    }
    else {
        return nullptr;
    }
}

PropertyBase::Ptr DOMPropertyMapLoader::readProperty (DOMElem::Ptr element, bool checkHeader)
{
    if (checkHeader)
        if (!element->isName (DOMPropertyMapFormat::idProperty ()))
            RW_THROW ("Parse error: Expected \"Property\" Got \"" + element->getName () + "\"!");

    std::string name = "", description = "";
    DOMElem::Ptr value = NULL;
    for (DOMElem::Ptr child : element->getChildren ()) {
        if (child->isName (DOMPropertyMapFormat::idPropertyName ())) {
            name = child->getValue ();
        }
        else if (child->isName (DOMPropertyMapFormat::idPropertyDescription ())) {
            description = child->getValue ();
        }
        else if (child->isName (DOMPropertyMapFormat::idPropertyValue ())) {
            value = child;
        }
        else {
            RW_THROW ("Parse Error: child element \" << child->getName() << \" not recognized in "
                      "Property with name \""
                      << name << "\"!");
        }
    }

    if (name == "")
        RW_THROW ("Parse Error: name element not defined in Property!");
    if (value == NULL)
        RW_THROW ("Parse Error: data value not defined in Property with name \"" << name << "\"!");

    for (DOMElem::Ptr child : value->getChildren ()) {
        PropertyValueBase::Ptr baseval = readPropertyValue(child);
        if (baseval.isNull()) {
            RW_THROW ("Parse Error: data value \""
                      << child->getName () << "\" not recognized in Property with name \"" << name
                      << "\"!");
        } else {
            if (child->isName (DOMPropertyMapFormat::idPropertyMap ())) {
                typedef PropertyMap Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMPropertyMapFormat::idPropertyValueList ())) {
                typedef std::vector< PropertyValueBase::Ptr > Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idString ())) {
                typedef std::string Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idStringList ())) {
                typedef std::vector< std::string > Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idIntList ())) {
                typedef std::vector< int > Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idDoubleList ())) {
                typedef std::vector< double > Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idDouble ())) {
                typedef double Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idFloat ())) {
                typedef float Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idInteger ())) {
                typedef int Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idBoolean ())) {
                typedef bool Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idVector3D ())) {
                typedef Vector3D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idVector2D ())) {
                typedef Vector2D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idQ ())) {
                typedef Q Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idTransform3D ())) {
                typedef Transform3D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idRotation3D ())) {
                typedef Rotation3D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idEAA ())) {
                typedef EAA<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idRPY ())) {
                typedef RPY<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idQuaternion ())) {
                typedef Quaternion<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idRotation2D ())) {
                typedef Rotation2D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMBasisTypes::idVelocityScrew6D ())) {
                typedef VelocityScrew6D<> Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMPathLoader::idQPath ())) {
                typedef QPath Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else if (child->isName (DOMPathLoader::idT3DPath ())) {
                typedef Transform3DPath Type;
                PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
                return ownedPtr (new Property< Type > (name, description, pval->getValue()));
            }
            else {
                return nullptr;
            }
        }
    }

    return NULL;
}

bool DOMPropertyMapLoader::hasProperties (DOMElem::Ptr element)
{
    return element->isName (DOMPropertyMapFormat::idPropertyMap ());
}

PropertyMap DOMPropertyMapLoader::readProperties (DOMElem::Ptr element, bool checkHeader)
{
    if (checkHeader)
        if (!element->isName (DOMPropertyMapFormat::idPropertyMap ()))
            RW_THROW ("Parse error: Expected \"PropertyMap\" got \"" + element->getName () + "\"!");

    PropertyMap properties;
    for (DOMElem::Ptr child : element->getChildren ()) {
        if (child->isName (DOMPropertyMapFormat::idProperty ())) {
            PropertyBase::Ptr property = readProperty (child, false);
            if (property != NULL)
                properties.add (property);
        }
    }

    return properties;
}

PropertyMap DOMPropertyMapLoader::load (std::istream& instream, const std::string& schemaFileName)
{
    DOMParser::Ptr parser = DOMParser::make ();
    // todo: add schema load to interface
    parser->load (instream);
    DOMElem::Ptr elementRoot = parser->getRootElement ();
    DOMElem::Ptr pmapRoot = elementRoot->getChild (DOMPropertyMapFormat::idPropertyMap (), false);
    PropertyMap map       = readProperties (pmapRoot);
    // map.set<std::string>("PropertyMapFileName", "");
    return map;
}

PropertyMap DOMPropertyMapLoader::load (const std::string& filename,
                                        const std::string& schemaFileName)
{
    DOMParser::Ptr parser = DOMParser::make ();
    // todo: add schema load to interface
    parser->load (filename);
    DOMElem::Ptr elementRoot = parser->getRootElement ();
    DOMElem::Ptr pmapRoot = elementRoot->getChild (DOMPropertyMapFormat::idPropertyMap (), false);
    PropertyMap map       = readProperties (pmapRoot);
    // map.set<std::string>("PropertyMapFileName", filename);
    return map;
}
