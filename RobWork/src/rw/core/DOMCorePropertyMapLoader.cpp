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

#include <rw/core/DOMCoreBasisTypes.hpp>
#include <rw/core/DOMCorePropertyMapLoader.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/core/DOMPropertyMapFormat.hpp>

using namespace rw::core;

DOMCorePropertyMapLoader::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        DOMCoreBasisTypes::Initializer init1;
        DOMPropertyMapFormat::Initializer init2;
        done = true;
    }
}

const DOMCorePropertyMapLoader::Initializer DOMCorePropertyMapLoader::initializer;

DOMCorePropertyMapLoader::DOMCorePropertyMapLoader ()
{}

DOMCorePropertyMapLoader::~DOMCorePropertyMapLoader ()
{}

PropertyBase::Ptr DOMCorePropertyMapLoader::readProperty (DOMElem::Ptr element, bool checkHeader)
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
        if (child->isName (DOMPropertyMapFormat::idPropertyMap ())) {
            return ownedPtr (new Property< PropertyMap > (
                name, description, DOMCorePropertyMapLoader::readProperties (child, true)));
        }
        else if (child->isName ("ValueList")) {
            RW_THROW("PropertyValueBasePtrList is not implemented in DOMCorePropertyMapLoader, Use DOMPropertyMapLoader instead");
            break;
        }
        else if (child->isName (DOMCoreBasisTypes::idString ())) {
            return ownedPtr (new Property< std::string > (
                name, description, DOMCoreBasisTypes::readString (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idStringList ())) {
            return ownedPtr (new Property< std::vector< std::string > > (
                name, description, DOMCoreBasisTypes::readStringList (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idIntList ())) {
            return ownedPtr (new Property< std::vector< int > > (
                name, description, DOMCoreBasisTypes::readIntList (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idDoubleList ())) {
            return ownedPtr (new Property< std::vector< double > > (
                name, description, DOMCoreBasisTypes::readDoubleList (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idDouble ())) {
            return ownedPtr (
                new Property< double > (name, description, DOMCoreBasisTypes::readDouble (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idFloat ())) {
            return ownedPtr (
                new Property< float > (name, description, DOMCoreBasisTypes::readFloat (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idInteger ())) {
            return ownedPtr (
                new Property< int > (name, description, DOMCoreBasisTypes::readInt (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idBoolean ())) {
            return ownedPtr (
                new Property< bool > (name, description, DOMCoreBasisTypes::readBool (child)));
        }
        else if (child->isName (DOMCoreBasisTypes::idVector3D ())) {
            RW_THROW ("Vector3D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idVector2D ())) {
            RW_THROW ("Vector2D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idQ ())) {
            RW_THROW ("Q is not implemented in DOMCorePropertyMapLoader, Use DOMPropertyMapLoader "
                      "instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idTransform3D ())) {
            RW_THROW ("TransForm3D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idRotation3D ())) {
            RW_THROW ("Rotation3D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idEAA ())) {
            RW_THROW ("EAA is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idRPY ())) {
            RW_THROW ("RPY is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idQuaternion ())) {
            RW_THROW ("Quaternion is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idRotation2D ())) {
            RW_THROW ("Rotation2D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName (DOMCoreBasisTypes::idVelocityScrew6D ())) {
            RW_THROW ("VelocityScrew6D is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName ("QPath")) {
            RW_THROW ("QPath is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else if (child->isName ("T3DPath")) {
            RW_THROW ("T3DPath is not implemented in DOMCorePropertyMapLoader, Use "
                      "DOMPropertyMapLoader instead");
            return NULL;
        }
        else {
            RW_THROW ("Parse Error: data value \""
                      << child->getName () << "\" not recognized in Property with name \"" << name
                      << "\"!");
        }
    }

    return NULL;
}

bool DOMCorePropertyMapLoader::hasProperties (DOMElem::Ptr element)
{
    return element->isName (DOMPropertyMapFormat::idPropertyMap ());
}

PropertyMap DOMCorePropertyMapLoader::readProperties (DOMElem::Ptr element, bool checkHeader)
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

PropertyMap DOMCorePropertyMapLoader::load (std::istream& instream,
                                            const std::string& schemaFileName)
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

PropertyMap DOMCorePropertyMapLoader::load (const std::string& filename,
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
