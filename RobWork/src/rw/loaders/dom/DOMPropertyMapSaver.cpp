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

#include <rw/core/DOMParser.hpp>
#include <rw/core/Property.hpp>
#include <rw/core/PropertyBase.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMPathSaver.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>

using namespace rw::core;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::trajectory;

DOMPropertyMapSaver::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        DOMBasisTypes::Initializer init1;
        DOMPropertyMapFormat::Initializer init2;
        DOMPathSaver::Initializer init3;
        done = true;
    }
}

const DOMPropertyMapSaver::Initializer DOMPropertyMapSaver::initializer;

void DOMPropertyMapSaver::save (const PropertyBase::Ptr property, DOMElem::Ptr parent)
{
    if (property->getType ().getId () == PropertyType::Unknown) {
        RW_WARN ("The property with name \""
                 << property->getIdentifier ()
                 << "\" has type 'Unknown' and was ignored as it can not be saved!");
        return;
    }

    DOMElem::Ptr root    = parent->addChild (DOMPropertyMapFormat::idProperty ());
    DOMElem::Ptr element = DOMBasisTypes::createElement (
        DOMPropertyMapFormat::idPropertyName (), property->getIdentifier (), root);

    if (!property->getDescription ().empty ()) {
        element = DOMBasisTypes::createElement (
            DOMPropertyMapFormat::idPropertyDescription (), property->getDescription (), root);
    }

    element = root->addChild (DOMPropertyMapFormat::idPropertyValue ());
    save(property->getPropertyValue(), element);
}

void DOMPropertyMapSaver::save (const rw::core::PropertyMap& map, DOMElem::Ptr parent)
{
    DOMElem::Ptr root = parent->addChild (DOMPropertyMapFormat::idPropertyMap ());

    std::pair< PropertyMap::iterator, PropertyMap::iterator > iterators = map.getProperties ();
    for (PropertyMap::iterator it = iterators.first; it != iterators.second; ++it) {
        save (*it, root);
    }
}

void DOMPropertyMapSaver::save (const rw::core::PropertyMap& map, const std::string& filename)
{
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make ();

    createDOMDocument (map, parser);
    parser->save (filename);
}

void DOMPropertyMapSaver::write (const rw::core::PropertyMap& map, std::ostream& outstream)
{
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make ();

    createDOMDocument (map, parser);
    parser->save (outstream);
}

DOMElem::Ptr DOMPropertyMapSaver::createDOMDocument (const rw::core::PropertyMap& map,
                                                     DOMParser::Ptr parser)
{
    DOMElem::Ptr doc = parser->getRootElement ();

    save (map, doc);

    return doc;
}

void DOMPropertyMapSaver::save (const PropertyValueBase& value, DOMElem::Ptr parent)
{
    if (value.getType ().getId () == PropertyType::Unknown) {
        RW_WARN ("PropertyValueBase type 'Unknown' and was ignored as it can not be saved!");
        return;
    }

    switch (value.getType ().getId ()) {
        case PropertyType::Unknown:
            // Already handled above
            break;
        case PropertyType::PropertyMap: {
            const PropertyValue< PropertyMap >* prop = dynamic_cast< const PropertyValue< PropertyMap >* > (&value);
            save (prop->getValue (), parent);
            break;
        }
        case PropertyType::PropertyMapPtr: {
            const PropertyValue< PropertyMap::Ptr >* prop = dynamic_cast< const PropertyValue< PropertyMap::Ptr >* > (&value);
            save(*prop->getValue(), parent);
            break;
        }
        case PropertyType::PropertyValueBasePtrList: {
            const PropertyValue< std::vector< PropertyValueBase::Ptr > >* prop = dynamic_cast< const PropertyValue< std::vector< PropertyValueBase::Ptr > >* > (&value);
            DOMElem::Ptr element = parent->addChild (DOMPropertyMapFormat::idPropertyValueList());
            for (const PropertyValueBase::Ptr& p : prop->getValue()) {
                save(*p, element);
            }
            break;
        }
        case PropertyType::String: {
            const PropertyValue< std::string >* prop = dynamic_cast< const PropertyValue< std::string >* > (&value);
            DOMBasisTypes::createString (prop->getValue (), parent);
            break;
        }
        case PropertyType::StringList: {
            const PropertyValue< std::vector< std::string > >* prop = dynamic_cast< const PropertyValue< std::vector< std::string > >* > (&value);
            DOMBasisTypes::createStringList (prop->getValue (), parent);
            break;
        }
        case PropertyType::Float: {
            const PropertyValue< float >* prop = dynamic_cast< const PropertyValue< float >* > (&value);
            DOMBasisTypes::createFloat (prop->getValue (), parent);
            break;
        }
        case PropertyType::Double: {
            const PropertyValue< double >* prop = dynamic_cast< const PropertyValue< double >* > (&value);
            DOMBasisTypes::createDouble (prop->getValue (), parent);
            break;
        }
        case PropertyType::Int: {
            const PropertyValue< int >* prop = dynamic_cast< const PropertyValue< int >* > (&value);
            DOMBasisTypes::createInteger (prop->getValue (), parent);
            break;
        }
        case PropertyType::Bool: {
            const PropertyValue< bool >* prop = dynamic_cast< const PropertyValue< bool >* > (&value);
            DOMBasisTypes::createBoolean (prop->getValue (), parent);
            break;
        }
        case PropertyType::Vector3D: {
            const PropertyValue< Vector3D<> >* prop = dynamic_cast< const PropertyValue< Vector3D<> >* > (&value);
            DOMBasisTypes::createVector3D (prop->getValue (), parent);
            break;
        }
        case PropertyType::Vector2D: {
            const PropertyValue< Vector2D<> >* prop = dynamic_cast< const PropertyValue< Vector2D<> >* > (&value);
            DOMBasisTypes::createVector2D (prop->getValue (), parent);
            break;
        }
        case PropertyType::Q: {
            const PropertyValue< Q >* prop = dynamic_cast< const PropertyValue< Q >* > (&value);
            DOMBasisTypes::createQ (prop->getValue (), parent);
            break;
        }
        case PropertyType::Transform3D: {
            const PropertyValue< Transform3D<> >* prop = dynamic_cast< const PropertyValue< Transform3D<> >* > (&value);
            DOMBasisTypes::createTransform3D (prop->getValue (), parent);
            break;
        }
        case PropertyType::Rotation3D: {
            const PropertyValue< Rotation3D<> >* prop = dynamic_cast< const PropertyValue< Rotation3D<> >* > (&value);
            DOMBasisTypes::createRotation3D (prop->getValue (), parent);
            break;
        }
        case PropertyType::EAA: {
            const PropertyValue< EAA<> >* prop = dynamic_cast< const PropertyValue< EAA<> >* > (&value);
            DOMBasisTypes::createEAA (prop->getValue (), parent);
            break;
        }
        case PropertyType::RPY: {
            const PropertyValue< RPY<> >* prop = dynamic_cast< const PropertyValue< RPY<> >* > (&value);
            DOMBasisTypes::createRPY (prop->getValue (), parent);
            break;
        }
        case PropertyType::Quaternion: {
            const PropertyValue< Quaternion<> >* prop = dynamic_cast< const PropertyValue< Quaternion<> >* > (&value);
            Quaternion<> normalizedQuaternion (prop->getValue ());
            normalizedQuaternion.normalize ();
            DOMBasisTypes::createQuaternion (normalizedQuaternion, parent);
            break;
        }
        case PropertyType::Rotation2D: {
            const PropertyValue< Rotation2D<> >* prop = dynamic_cast< const PropertyValue< Rotation2D<> >* > (&value);
            DOMBasisTypes::createRotation2D (prop->getValue (), parent);
            break;
        }
        case PropertyType::VelocityScrew6D: {
            const PropertyValue< VelocityScrew6D<> >* prop = dynamic_cast< const PropertyValue< VelocityScrew6D<> >* > (&value);
            DOMBasisTypes::createVelocityScrew6D (prop->getValue (), parent);
            break;
        }
        case PropertyType::IntList: {
            const PropertyValue< std::vector< int > >* prop = dynamic_cast< const PropertyValue< std::vector< int > >* > (&value);
            DOMBasisTypes::createIntList (prop->getValue (), parent);
            break;
        }
        case PropertyType::DoubleList: {
            const PropertyValue< std::vector< double > >* prop = dynamic_cast< const PropertyValue< std::vector< double > >* > (&value);
            DOMBasisTypes::createDoubleList (prop->getValue (), parent);
            break;
        }
        case PropertyType::QPath: {
            const PropertyValue< QPath >* prop = dynamic_cast< const PropertyValue< QPath >* > (&value);
            DOMPathSaver::createQPath (prop->getValue (), parent);
            break;
        }
        case PropertyType::QPathPtr: {
            const PropertyValue< QPath::Ptr >* prop = dynamic_cast< const PropertyValue< QPath::Ptr >* > (&value);
            DOMPathSaver::createQPath (*prop->getValue (), parent);
            break;
        }
        case PropertyType::Transform3DPath: {
            const PropertyValue< Transform3DPath >* prop = dynamic_cast< const PropertyValue< Transform3DPath >* > (&value);
            DOMPathSaver::createTransform3DPath (prop->getValue (), parent);
            break;
        }
        case PropertyType::Transform3DPathPtr: {
            const PropertyValue< Transform3DPath::Ptr >* prop = dynamic_cast< const PropertyValue< Transform3DPath::Ptr >* > (&value);
            DOMPathSaver::createTransform3DPath (*prop->getValue (), parent);
            break;
        }
        default:
            RW_THROW ("The property type has no save implementation within DOMPropertyMapSaver!");
    }    // end switch(property.getType)
}
