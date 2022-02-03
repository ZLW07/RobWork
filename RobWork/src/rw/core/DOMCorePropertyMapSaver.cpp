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
#include <rw/core/DOMCorePropertyMapSaver.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/core/DOMPropertyMapFormat.hpp>
#include <rw/core/Property.hpp>
#include <rw/core/PropertyBase.hpp>
#include <rw/core/PropertyMap.hpp>

using namespace rw::core;

DOMCorePropertyMapSaver::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        DOMCoreBasisTypes::Initializer init1;
        DOMPropertyMapFormat::Initializer init2;
        done = true;
    }
}

const DOMCorePropertyMapSaver::Initializer DOMCorePropertyMapSaver::initializer;

void DOMCorePropertyMapSaver::save (const PropertyBase::Ptr property, DOMElem::Ptr parent)
{
    if (property->getType ().getId () == PropertyType::Unknown) {
        RW_WARN ("The property with name \""
                 << property->getIdentifier ()
                 << "\" has type 'Unknown' and was ignored as it can not be saved!");
        return;
    }

    DOMElem::Ptr root    = parent->addChild (DOMPropertyMapFormat::idProperty ());
    DOMElem::Ptr element = DOMCoreBasisTypes::createElement (
        DOMPropertyMapFormat::idPropertyName (), property->getIdentifier (), root);

    if (!property->getDescription ().empty ()) {
        element = DOMCoreBasisTypes::createElement (
            DOMPropertyMapFormat::idPropertyDescription (), property->getDescription (), root);
    }

    element = root->addChild (DOMPropertyMapFormat::idPropertyValue ());
    switch (property->getType ().getId ()) {
        case PropertyType::Unknown:
            // Already handled above
            break;
        case PropertyType::PropertyMap: {
            const Property< PropertyMap >* prop = toProperty< PropertyMap > (property);
            save (prop->getValue (), element);
            break;
        }
        case PropertyType::PropertyMapPtr: {
            const Property<PropertyMap::Ptr>* prop = toProperty<PropertyMap::Ptr>(property);
            save(*prop->getValue(), element);
            break;
        }
        case PropertyType::PropertyValueBasePtrList: {
            RW_THROW("PropertyValueBasePtrList is not implemented in DOMCorePropertyMapSaver, Use DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::String: {
            const Property< std::string >* prop = toProperty< std::string > (property);
            DOMCoreBasisTypes::createString (prop->getValue (), element);
            break;
        }
        case PropertyType::StringList: {
            const Property< std::vector< std::string > >* prop =
                toProperty< std::vector< std::string > > (property);
            DOMCoreBasisTypes::createStringList (prop->getValue (), element);
            break;
        }
        case PropertyType::Float: {
            const Property< float >* prop = toProperty< float > (property);
            DOMCoreBasisTypes::createFloat (prop->getValue (), element);
            break;
        }
        case PropertyType::Double: {
            const Property< double >* prop = toProperty< double > (property);
            DOMCoreBasisTypes::createDouble (prop->getValue (), element);
            break;
        }
        case PropertyType::Int: {
            const Property< int >* prop = toProperty< int > (property);
            DOMCoreBasisTypes::createInteger (prop->getValue (), element);
            break;
        }
        case PropertyType::Bool: {
            const Property< bool >* prop = toProperty< bool > (property);
            DOMCoreBasisTypes::createBoolean (prop->getValue (), element);
            break;
        }
        case PropertyType::Vector3D: {
            RW_THROW ("Vector3D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Vector2D: {
            RW_THROW ("Vector2D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Q: {
            RW_THROW (
                "Q is not implemented in DOMCorePropertyMapSaver, Use DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Transform3D: {
            RW_THROW ("Traqnsform3D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Rotation3D: {
            RW_THROW ("Rotation3D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::EAA: {
            RW_THROW ("EAA is not implemented in DOMCorePropertyMapSaver, Use DOMPropertyMapSaver "
                      "instead");
            break;
        }
        case PropertyType::RPY: {
            RW_THROW ("RPY is not implemented in DOMCorePropertyMapSaver, Use DOMPropertyMapSaver "
                      "instead");
            break;
        }
        case PropertyType::Quaternion: {
            RW_THROW ("Quaternion is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Rotation2D: {
            RW_THROW ("Rotation2D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::VelocityScrew6D: {
            RW_THROW ("VelocityScrew6D is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::IntList: {
            const Property< std::vector< int > >* prop =
                toProperty< std::vector< int > > (property);
            DOMCoreBasisTypes::createIntList (prop->getValue (), element);
            break;
        }
        case PropertyType::DoubleList: {
            const Property< std::vector< double > >* prop =
                toProperty< std::vector< double > > (property);
            DOMCoreBasisTypes::createDoubleList (prop->getValue (), element);
            break;
        }
        case PropertyType::QPath: {
            RW_THROW ("QPath is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::QPathPtr: {
            RW_THROW ("QPathPtr is not implemented in DOMCorePropertyMapSaver, "
                      "Use DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Transform3DPath: {
            RW_THROW ("Transform3DPath is not implemented in DOMCorePropertyMapSaver, Use "
                      "DOMPropertyMapSaver instead");
            break;
        }
        case PropertyType::Transform3DPathPtr: {
            RW_THROW ("Transform3DPathPtr is not implemented in "
                      "DOMCorePropertyMapSaver, Use DOMPropertyMapSaver "
                      "instead");
            break;
        }
        default:
            RW_THROW (
                "The property type has no save implementation within DOMCorePropertyMapSaver!");
    }    // end switch(property.getType)
}

void DOMCorePropertyMapSaver::save (const rw::core::PropertyMap& map, DOMElem::Ptr parent)
{
    DOMElem::Ptr root = parent->addChild (DOMPropertyMapFormat::idPropertyMap ());

    std::pair< PropertyMap::iterator, PropertyMap::iterator > iterators = map.getProperties ();
    for (PropertyMap::iterator it = iterators.first; it != iterators.second; ++it) {
        save (*it, root);
    }
}

void DOMCorePropertyMapSaver::save (const rw::core::PropertyMap& map, const std::string& filename)
{
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make ();

    createDOMDocument (map, parser);
    parser->save (filename);
}

void DOMCorePropertyMapSaver::write (const rw::core::PropertyMap& map, std::ostream& outstream)
{
    /* DOMParser::make() as of this writing returns the default XML parser */
    DOMParser::Ptr parser = DOMParser::make ();

    createDOMDocument (map, parser);
    parser->save (outstream);
}

DOMElem::Ptr DOMCorePropertyMapSaver::createDOMDocument (const rw::core::PropertyMap& map,
                                                         DOMParser::Ptr parser)
{
    DOMElem::Ptr doc = parser->getRootElement ();

    save (map, doc);

    return doc;
}
