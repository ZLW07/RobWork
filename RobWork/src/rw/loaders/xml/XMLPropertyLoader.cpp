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

#include "XMLPropertyLoader.hpp"

#include "XMLPropertyFormat.hpp"
#include "XercesUtils.hpp"

#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPathFormat.hpp>
#include <rw/loaders/xml/XMLPathLoader.hpp>
#include <rw/loaders/xml/XercesErrorHandler.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLDouble.hpp>
#include <xercesc/util/XMLUni.hpp>

using namespace rw::math;
using namespace rw::core;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace xercesc;

XMLPropertyLoader::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        XMLBasisTypes::Initializer init1;
        XMLPropertyFormat::Initializer init2;
        XMLPathFormat::Initializer init3;
        XMLPathLoader::Initializer init4;
        done = true;
    }
}

const XMLPropertyLoader::Initializer XMLPropertyLoader::initializer;

XMLPropertyLoader::XMLPropertyLoader ()
{}

XMLPropertyLoader::~XMLPropertyLoader ()
{}

namespace {

xercesc::DOMElement* getChildElement (xercesc::DOMElement* element, bool throwOnEmpty)
{
    DOMNodeList* children = element->getChildNodes ();
    for (size_t i = 0; i < children->getLength (); i++) {
        xercesc::DOMElement* element = dynamic_cast< xercesc::DOMElement* > (children->item (i));
        if (element != NULL)
            return element;
    }
    if (throwOnEmpty)
        RW_THROW ("No child element to node " + XMLStr (element->getNodeName ()).str ());
    return NULL;
}

PropertyBase::Ptr getProperty (const std::string& name, const std::string& description, int type2,
        PropertyValueBase::Ptr baseval, xercesc::DOMElement* valueNode)
{
    xercesc::DOMElement* child = getChildElement(valueNode, false);
    if (child == nullptr)
        return nullptr;

    if (XMLString::equals (child->getNodeName (), XMLPropertyFormat::idPropertyMap ())) {
        typedef PropertyMap Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLPropertyFormat::idPropertyValueList ())) {
        typedef std::vector< PropertyValueBase::Ptr > Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idString ())) {
        typedef std::string Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idStringList ())) {
        typedef std::vector< std::string > Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idIntList ())) {
        typedef std::vector< int > Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idDoubleList ())) {
        typedef std::vector< double > Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idDouble ())) {
        typedef double Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idFloat ())) {
        typedef float Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idInteger ())) {
        typedef int Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idBoolean ())) {
        typedef bool Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVector3D ())) {
        typedef Vector3D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVector2D ())) {
        typedef Vector2D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idQ ())) {
        typedef Q Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idTransform3D ())) {
        typedef Transform3D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRotation3D ())) {
        typedef Rotation3D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idEAA ())) {
        typedef EAA<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRPY ())) {
        typedef RPY<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idQuaternion ())) {
        typedef Quaternion<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRotation2D ())) {
        typedef Rotation2D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVelocityScrew6D ())) {
        typedef VelocityScrew6D<> Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLPathFormat::idQPath ())) {
        typedef QPath Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else if (XMLString::equals (child->getNodeName (), XMLPathFormat::idT3DPath ())) {
        typedef Transform3DPath Type;
        PropertyValue< Type >::Ptr pval = baseval.scast<PropertyValue< Type > >();
        return ownedPtr (new Property< Type > (name, description, pval->getValue()));
    }
    else {
        return nullptr;
    }
}

}    // namespace

PropertyValueBase::Ptr XMLPropertyLoader::readPropertyValue (xercesc::DOMElement* valueNode)
{
    xercesc::DOMElement* child = getChildElement(valueNode, false);
    if (child == nullptr)
        return nullptr;

    if (XMLString::equals (child->getNodeName (), XMLPropertyFormat::idPropertyMap ()))
        return ownedPtr (new PropertyValue< PropertyMap > (
            XMLPropertyLoader::readProperties (child, true)));
    if (XMLString::equals (child->getNodeName (), XMLPropertyFormat::idPropertyValueList ())) {
        std::vector<PropertyValueBase::Ptr> list;
        DOMNodeList* children     = child->getChildNodes ();
        const XMLSize_t nodeCount = children->getLength ();
        for (XMLSize_t i = 0; i < nodeCount; ++i) {
            xercesc::DOMElement* child = dynamic_cast< xercesc::DOMElement* > (children->item (i));
            list.push_back(readPropertyValue(child));
        }
        return ownedPtr (new PropertyValue< std::vector< PropertyValueBase::Ptr > > (list));
    }
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idString ()))
        return ownedPtr (
            new PropertyValue< std::string > (XMLBasisTypes::readString (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idStringList ()))
        return ownedPtr (new PropertyValue< std::vector< std::string > > (
            XMLBasisTypes::readStringList (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idIntList ()))
        return ownedPtr (new PropertyValue< std::vector< int > > (
            XMLBasisTypes::readIntList (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idDoubleList ()))
        return ownedPtr (new PropertyValue< std::vector< double > > (
            XMLBasisTypes::readDoubleList (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idDouble ()))
        return ownedPtr (
            new PropertyValue< double > (XMLBasisTypes::readDouble (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idFloat ()))
        return ownedPtr (
            new PropertyValue< float > (XMLBasisTypes::readFloat (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idInteger ()))
        return ownedPtr (new PropertyValue< int > (XMLBasisTypes::readInt (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idBoolean ()))
        return ownedPtr (new PropertyValue< bool > (XMLBasisTypes::readBool (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVector3D ()))
        return ownedPtr (
            new PropertyValue< Vector3D<> > (XMLBasisTypes::readVector3D (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVector2D ()))
        return ownedPtr (
            new PropertyValue< Vector2D<> > (XMLBasisTypes::readVector2D (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idQ ()))
        return ownedPtr (new PropertyValue< Q > (XMLBasisTypes::readQ (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idTransform3D ()))
        return ownedPtr (new PropertyValue< Transform3D<> > (
            XMLBasisTypes::readTransform3D (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRotation3D ()))
        return ownedPtr (new PropertyValue< Rotation3D<> > (
            XMLBasisTypes::readRotation3D (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idEAA ()))
        return ownedPtr (new PropertyValue< EAA<> > (XMLBasisTypes::readEAA (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRPY ()))
        return ownedPtr (new PropertyValue< RPY<> > (XMLBasisTypes::readRPY (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idQuaternion ()))
        return ownedPtr (new PropertyValue< Quaternion<> > (
            XMLBasisTypes::readQuaternion (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idRotation2D ()))
        return ownedPtr (new PropertyValue< Rotation2D<> > (
            XMLBasisTypes::readRotation2D (child)));
    if (XMLString::equals (child->getNodeName (), XMLBasisTypes::idVelocityScrew6D ()))
        return ownedPtr (new PropertyValue< VelocityScrew6D<> > (
            XMLBasisTypes::readVelocityScrew6D (child)));
    if (XMLString::equals (child->getNodeName (), XMLPathFormat::idQPath ())) {
        XMLPathLoader loader (child);
        return ownedPtr (new PropertyValue< QPath > (*loader.getQPath ()));
    }
    if (XMLString::equals (child->getNodeName (), XMLPathFormat::idT3DPath ())) {
        XMLPathLoader loader (child);
        return ownedPtr (
            new PropertyValue< Transform3DPath > (*loader.getTransform3DPath ()));
    }
    return nullptr;
}

PropertyBase::Ptr XMLPropertyLoader::readProperty (xercesc::DOMElement* element, bool checkHeader)
{
    // std::cout<<"Read Property"<<std::endl;
    if (checkHeader)
        if (!XMLString::equals (XMLPropertyFormat::idProperty (), element->getNodeName ()))
            RW_THROW ("Element name does not match " +
                      XMLStr (XMLPropertyFormat::idProperty ()).str () + " as expected");

    DOMNodeList* children     = element->getChildNodes ();
    const XMLSize_t nodeCount = children->getLength ();

    std::string name                  = "";
    std::string description           = "";
    int type                          = -1;
    xercesc::DOMElement* valueElement = NULL;
    // First we run through and finds the interpolators
    for (XMLSize_t i = 0; i < nodeCount; ++i) {
        xercesc::DOMElement* child = dynamic_cast< xercesc::DOMElement* > (children->item (i));
        if (child != NULL) {
            if (XMLString::equals (XMLPropertyFormat::idPropertyName (), child->getNodeName ())) {
                name = XMLBasisTypes::readString (child);
            }
            else if (XMLString::equals (XMLPropertyFormat::idPropertyDescription (),
                                        child->getNodeName ())) {
                description = XMLBasisTypes::readString (child);
            }
            else if (XMLString::equals (XMLPropertyFormat::idPropertyType (),
                                        child->getNodeName ())) {
                type = XMLBasisTypes::readInt (child);
            }
            else if (XMLString::equals (XMLPropertyFormat::idPropertyValue (),
                                        child->getNodeName ())) {
                valueElement = child;
            }
        }
    }

    /*if (type == -1) {
            RW_WARN("Unable to find type of property " << name);
//    RW_THROW("Unable to find type of property " + name);
            return NULL;
    }*/

    if (name == "")
        RW_THROW ("Unable to find name of property");
    if (valueElement == NULL)
        RW_THROW ("Unable to find value of property " + name);

    const PropertyValueBase::Ptr baseval = readPropertyValue (valueElement);
    if (baseval.isNull())
        return nullptr;

    return getProperty (name, description, type, baseval, valueElement);
}

PropertyMap XMLPropertyLoader::readProperties (xercesc::DOMElement* element, bool checkHeader)
{
    if (checkHeader)
        if (!XMLString::equals (XMLPropertyFormat::idPropertyMap (), element->getNodeName ()))
            RW_THROW ("Element name does not match " +
                      XMLStr (XMLPropertyFormat::idPropertyMap ()).str () + " as expected");

    PropertyMap properties;
    DOMNodeList* children     = element->getChildNodes ();
    const XMLSize_t nodeCount = children->getLength ();

    // First we run through and finds the interpolators
    for (XMLSize_t i = 0; i < nodeCount; ++i) {
        xercesc::DOMElement* element = dynamic_cast< xercesc::DOMElement* > (children->item (i));
        if (element != NULL) {
            if (XMLString::equals (XMLPropertyFormat::idProperty (), element->getNodeName ())) {
                PropertyBase::Ptr property = readProperty (element, false);
                if (property != NULL)
                    properties.add (property);
            }
        }
    }

    return properties;
}

PropertyMap XMLPropertyLoader::load (std::istream& instream, const std::string& schemaFileName)
{
    XercesDOMParser parser;
    xercesc::DOMDocument* doc =
        XercesDocumentReader::readDocument (parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement ();
    PropertyMap map                  = readProperties (elementRoot);
    // map.set<std::string>("PropertyMapFileName", "");
    return map;
}

PropertyMap XMLPropertyLoader::load (const std::string& filename, const std::string& schemaFileName)
{
    XercesDOMParser parser;
    xercesc::DOMDocument* doc =
        XercesDocumentReader::readDocument (parser, filename, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement ();
    PropertyMap map                  = readProperties (elementRoot);
    // map.set<std::string>("PropertyMapFileName", filename);
    return map;
}
