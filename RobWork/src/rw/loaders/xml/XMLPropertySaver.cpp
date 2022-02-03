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

/*
 * XMLPropertySaver.cpp
 *
 *  Created on: Jan 5, 2009
 *      Author: lpe
 */

#include "XMLPropertySaver.hpp"

#include "XMLPropertyFormat.hpp"

#include <rw/core/PropertyValueBase.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPathSaver.hpp>
#include <rw/loaders/xml/XercesUtils.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

using namespace rw::math;
using namespace rw::core;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace xercesc;

XMLPropertySaver::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        XMLBasisTypes::Initializer init1;
        XMLPropertyFormat::Initializer init2;
        XMLPathFormat::Initializer init3;
        XMLPathSaver::Initializer init4;
        done = true;
    }
}

const XMLPropertySaver::Initializer XMLPropertySaver::initializer;

xercesc::DOMElement* XMLPropertySaver::save (PropertyBase::Ptr property, xercesc::DOMDocument* doc)
{
    if (property->getType ().getId () == PropertyType::Unknown) {
        RW_WARN ("Ignoring property \"" << property->getIdentifier ()
                                        << "\" which has unknown property type!");
        return NULL;
    }

    xercesc::DOMElement* root = doc->createElement (XMLPropertyFormat::idProperty ());

    xercesc::DOMElement* element = doc->createElement (XMLPropertyFormat::idPropertyName ());
    root->appendChild (element);
    DOMText* txt = doc->createTextNode (XMLStr (property->getIdentifier ()).uni ());
    element->appendChild (txt);

    if (property->getDescription ().size () > 0) {
        element = doc->createElement (XMLPropertyFormat::idPropertyDescription ());
        root->appendChild (element);
        txt = doc->createTextNode (XMLStr (property->getDescription ()).uni ());
        element->appendChild (txt);
    }
    // element = doc->createElement(XMLPropertyFormat::PropertyTypeId);
    // root->appendChild(element);
    // txt = doc->createTextNode(XMLStr(property->getType().getId()).uni());
    // element->appendChild(txt);

    element = doc->createElement (XMLPropertyFormat::idPropertyValue ());
    root->appendChild (element);

    save(property->getPropertyValue(), element, doc);

    return root;
}

void XMLPropertySaver::save (const rw::core::PropertyMap& map, xercesc::DOMElement* parent,
                             xercesc::DOMDocument* doc)
{
    std::pair< PropertyMap::iterator, PropertyMap::iterator > iterators = map.getProperties ();
    for (PropertyMap::iterator it = iterators.first; it != iterators.second; ++it) {
        xercesc::DOMElement* element = save (*it, doc);
        if (element != NULL)
            parent->appendChild (element);
    }
}

xercesc::DOMElement* XMLPropertySaver::save (const PropertyMap& map, xercesc::DOMDocument* doc)
{
    xercesc::DOMElement* element = doc->createElement (XMLPropertyFormat::idPropertyMap ());
    save (map, element, doc);
    return element;
}

xercesc::DOMDocument* XMLPropertySaver::createDOMDocument (const PropertyMap& map)
{
    XMLCh* features         = XMLString::transcode ("Core");
    DOMImplementation* impl = DOMImplementationRegistry::getDOMImplementation (features);
    XMLString::release (&features);
    xercesc::DOMDocument* doc = NULL;
    if (impl != NULL) {
        try {
            doc = impl->createDocument (0,    // root element namespace URI.
                                        XMLPropertyFormat::idPropertyMap (),    // root element name
                                        0);    // We do not wish to specify a document type

            xercesc::DOMElement* root = doc->getDocumentElement ();

            // Call the save method
            save (map, root, doc);
        }
        catch (const OutOfMemoryException&) {
            RW_THROW ("XMLPropertySaver: OutOfMemory");
        }
        catch (const DOMException& e) {
            RW_THROW ("XMLPropertySaver: DOMException:  " << XMLStr (e.getMessage ()).str ());
        }
        catch (const rw::core::Exception& exp) {
            throw exp;
        }
        catch (...) {
            RW_THROW ("XMLPropertySaver: Unknown Exception while creating saving path");
        }
    }
    else {
        RW_THROW ("XMLPropertySaver: Unable to find a suitable DOM Implementation");
    }
    return doc;
}

void XMLPropertySaver::save (const rw::core::PropertyMap& map, const std::string& filename)
{
    xercesc::DOMDocument* doc = createDOMDocument (map);
    XercesDocumentWriter::writeDocument (doc, filename);
    doc->release ();
}

void XMLPropertySaver::write (const rw::core::PropertyMap& map, std::ostream& outstream)
{
    xercesc::DOMDocument* doc = createDOMDocument (map);
    XercesDocumentWriter::writeDocument (doc, outstream);
    doc->release ();
}

void XMLPropertySaver::save (
    const PropertyValueBase& property,
    xercesc::DOMElement* parent, xercesc::DOMDocument* doc)
{
    xercesc::DOMElement* elem = NULL;
    switch (property.getType ().getId ()) {
        case PropertyType::Unknown: RW_WARN ("Unable to save property of unknown type"); break;
        case PropertyType::PropertyMap: {
            const PropertyValue< PropertyMap >* prop =
                dynamic_cast< const PropertyValue< PropertyMap >* > (&property);
            elem = save (prop->getValue (), doc);
            break;
        }
        case PropertyType::PropertyValueBasePtrList: {
            const PropertyValue< std::vector< PropertyValueBase::Ptr > >* prop =
                    dynamic_cast< const PropertyValue< std::vector< PropertyValueBase::Ptr > >* > (&property);
            elem = doc->createElement (XMLPropertyFormat::idPropertyValueList());
            for (const PropertyValueBase::Ptr& p : prop->getValue()) {
                save(*p, elem, doc);
            }
            break;
        }
        case PropertyType::String: {
            const PropertyValue< std::string >* prop =
                dynamic_cast< const PropertyValue< std::string >* > (&property);
            elem = XMLBasisTypes::createString (prop->getValue (), doc);
            break;
        }
        case PropertyType::StringList: {
            const PropertyValue< std::vector< std::string > >* prop =
                dynamic_cast< const PropertyValue< std::vector< std::string > >* > (&property);
            elem = XMLBasisTypes::createStringList (prop->getValue (), doc);
            break;
        }
        case PropertyType::Float: {
            const PropertyValue< float >* prop =
                dynamic_cast< const PropertyValue< float >* > (&property);
            elem = XMLBasisTypes::createDouble (prop->getValue (), doc);
            break;
        }
        case PropertyType::Double: {
            const PropertyValue< double >* prop =
                dynamic_cast< const PropertyValue< double >* > (&property);
            elem = XMLBasisTypes::createDouble (prop->getValue (), doc);
            break;
        }
        case PropertyType::Int: {
            const PropertyValue< int >* prop = dynamic_cast< const PropertyValue< int >* > (&property);
            elem                        = XMLBasisTypes::createInteger (prop->getValue (), doc);
            break;
        }
        case PropertyType::Bool: {
            const PropertyValue< bool >* prop =
                dynamic_cast< const PropertyValue< bool >* > (&property);
            elem = XMLBasisTypes::createBoolean (prop->getValue (), doc);
            break;
        }
        case PropertyType::Vector3D: {
            const PropertyValue< Vector3D<> >* prop =
                dynamic_cast< const PropertyValue< Vector3D<> >* > (&property);
            elem = XMLBasisTypes::createVector3D (prop->getValue (), doc);
            break;
        }
        case PropertyType::Vector2D: {
            const PropertyValue< Vector2D<> >* prop =
                dynamic_cast< const PropertyValue< Vector2D<> >* > (&property);
            elem = XMLBasisTypes::createVector2D (prop->getValue (), doc);
            break;
        }
        case PropertyType::Q: {
            const PropertyValue< Q >* prop = dynamic_cast< const PropertyValue< Q >* > (&property);
            elem                      = XMLBasisTypes::createQ (prop->getValue (), doc);
            break;
        }
        case PropertyType::Transform3D: {
            const PropertyValue< Transform3D<> >* prop =
                dynamic_cast< const PropertyValue< Transform3D<> >* > (&property);
            elem = XMLBasisTypes::createTransform3D (prop->getValue (), doc);
            break;
        }
        case PropertyType::Rotation3D: {
            const PropertyValue< Rotation3D<> >* prop =
                dynamic_cast< const PropertyValue< Rotation3D<> >* > (&property);
            elem = XMLBasisTypes::createRotation3D (prop->getValue (), doc);
            break;
        }
        case PropertyType::EAA: {
            const PropertyValue< EAA<> >* prop =
                dynamic_cast< const PropertyValue< EAA<> >* > (&property);
            elem = XMLBasisTypes::createEAA (prop->getValue (), doc);
            break;
        }
        case PropertyType::RPY: {
            const PropertyValue< RPY<> >* prop =
                dynamic_cast< const PropertyValue< RPY<> >* > (&property);
            elem = XMLBasisTypes::createRPY (prop->getValue (), doc);
            break;
        }
        case PropertyType::Quaternion: {
            const PropertyValue< Quaternion<> >* prop =
                dynamic_cast< const PropertyValue< Quaternion<> >* > (&property);
            elem = XMLBasisTypes::createQuaternion (prop->getValue (), doc);
            break;
        }
        case PropertyType::Rotation2D: {
            const PropertyValue< Rotation2D<> >* prop =
                dynamic_cast< const PropertyValue< Rotation2D<> >* > (&property);
            elem = XMLBasisTypes::createRotation2D (prop->getValue (), doc);
            break;
        }
        case PropertyType::VelocityScrew6D: {
            const PropertyValue< VelocityScrew6D<> >* prop =
                dynamic_cast< const PropertyValue< VelocityScrew6D<> >* > (&property);
            elem = XMLBasisTypes::createVelocityScrew6D (prop->getValue (), doc);
            break;
        }
        case PropertyType::QPath: {
            const PropertyValue< QPath >* prop =
                dynamic_cast< const PropertyValue< QPath >* > (&property);
            elem = XMLPathSaver::createElement< Q, QPath > (
                prop->getValue (), XMLPathFormat::idQPath (), doc);
            break;
        }
        case PropertyType::Transform3DPath: {
            const PropertyValue< Transform3DPath >* prop =
                dynamic_cast< const PropertyValue< Transform3DPath >* > (&property);
            elem = XMLPathSaver::createElement< Transform3D<>, Transform3DPath > (
                prop->getValue (), XMLPathFormat::idT3DPath (), doc);
            break;
        }
        case PropertyType::IntList: {
            const PropertyValue< std::vector< int > >* prop =
                dynamic_cast< const PropertyValue< std::vector< int > >* > (&property);
            elem = XMLBasisTypes::createIntList (prop->getValue (), doc);
            break;
        }
        case PropertyType::DoubleList: {
            const PropertyValue< std::vector< double > >* prop =
                dynamic_cast< const PropertyValue< std::vector< double > >* > (&property);
            elem = XMLBasisTypes::createDoubleList (prop->getValue (), doc);
            break;
        }
    }    // end switch(property.getType)

    if (elem != NULL)
        parent->appendChild (elem);
}
