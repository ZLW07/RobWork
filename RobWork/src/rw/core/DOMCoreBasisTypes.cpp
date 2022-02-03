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

#include "DOMCoreBasisTypes.hpp"

#include <rw/core/DOMElem.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/core/macros.hpp>

#include <boost/lexical_cast.hpp>
#include <map>
#include <sstream>
#include <vector>

using namespace rw::core;

DOMCoreBasisTypes::Initializer::Initializer ()
{
    static bool done = false;
    if (!done) {
        idQ ();
        idVector3D ();
        idVector2D ();
        idRotation3D ();
        idRPY ();
        idEAA ();
        idQuaternion ();
        idRotation2D ();
        idRotation2DAngle ();
        idTransform2D ();
        idTransform3D ();
        idMatrix ();
        idVelocityScrew6D ();
        idPos ();
        idLinear ();
        idAngular ();
        idState ();
        idQState ();
        idTreeState ();
        idBoolean ();
        idDouble ();
        idFloat ();
        idInteger ();
        idString ();
        idStringList ();
        idIntList ();
        idDoubleList ();
        idStringPair ();
        idUnitAttribute ();
        done = true;
    }
}

const DOMCoreBasisTypes::Initializer DOMCoreBasisTypes::initializer;

// Definition of Identifiers used in the XML format
const std::string& DOMCoreBasisTypes::idQ ()
{
    static const std::string id ("Q");
    return id;
}

const std::string& DOMCoreBasisTypes::idVector3D ()
{
    static const std::string id ("Vector3D");
    return id;
}

const std::string& DOMCoreBasisTypes::idVector2D ()
{
    static const std::string id ("Vector2D");
    return id;
}

const std::string& DOMCoreBasisTypes::idRotation3D ()
{
    static const std::string id ("Rotation3D");
    return id;
}

const std::string& DOMCoreBasisTypes::idRPY ()
{
    static const std::string id ("RPY");
    return id;
}

const std::string& DOMCoreBasisTypes::idEAA ()
{
    static const std::string id ("EAA");
    return id;
}

const std::string& DOMCoreBasisTypes::idQuaternion ()
{
    static const std::string id ("Quaternion");
    return id;
}

const std::string& DOMCoreBasisTypes::idRotation2D ()
{
    static const std::string id ("Rotation2D");
    return id;
}

const std::string& DOMCoreBasisTypes::idRotation2DAngle ()
{
    static const std::string id ("Rotation2DAngle");
    return id;
}

const std::string& DOMCoreBasisTypes::idTransform2D ()
{
    static const std::string id ("Transform2D");
    return id;
}

const std::string& DOMCoreBasisTypes::idTransform3D ()
{
    static const std::string id ("Transform3D");
    return id;
}

const std::string& DOMCoreBasisTypes::idMatrix ()
{
    static const std::string id ("Matrix");
    return id;
}

const std::string& DOMCoreBasisTypes::idVelocityScrew6D ()
{
    static const std::string id ("VelocityScrew6D");
    return id;
}

const std::string& DOMCoreBasisTypes::idPos ()
{
    static const std::string id ("Pos");
    return id;
}

const std::string& DOMCoreBasisTypes::idLinear ()
{
    static const std::string id ("Linear");
    return id;
}

const std::string& DOMCoreBasisTypes::idAngular ()
{
    static const std::string id ("Angular");
    return id;
}

const std::string& DOMCoreBasisTypes::idState ()
{
    static const std::string id ("State");
    return id;
}

const std::string& DOMCoreBasisTypes::idQState ()
{
    static const std::string id ("QState");
    return id;
}

const std::string& DOMCoreBasisTypes::idTreeState ()
{
    static const std::string id ("TreeState");
    return id;
}

const std::string& DOMCoreBasisTypes::idBoolean ()
{
    static const std::string id ("Boolean");
    return id;
}

const std::string& DOMCoreBasisTypes::idDouble ()
{
    static const std::string id ("Double");
    return id;
}

const std::string& DOMCoreBasisTypes::idFloat ()
{
    static const std::string id ("Float");
    return id;
}

const std::string& DOMCoreBasisTypes::idInteger ()
{
    static const std::string id ("Integer");
    return id;
}

const std::string& DOMCoreBasisTypes::idString ()
{
    static const std::string id ("String");
    return id;
}

const std::string& DOMCoreBasisTypes::idStringList ()
{
    static const std::string id ("StringList");
    return id;
}

const std::string& DOMCoreBasisTypes::idIntList ()
{
    static const std::string id ("IntList");
    return id;
}

const std::string& DOMCoreBasisTypes::idDoubleList ()
{
    static const std::string id ("DoubleList");
    return id;
}

const std::string& DOMCoreBasisTypes::idStringPair ()
{
    static const std::string id ("StringPair");
    return id;
}

const std::string& DOMCoreBasisTypes::idUnitAttribute ()
{
    static const std::string id ("unit");
    return id;
}

namespace {
struct UnitMap
{
  public:
    std::map< std::string, double > _map;

    UnitMap ()
    {
        double Deg2Rad = 3.1415926535897932384626433832795 / 180.0;
        _map["mm"]     = 1.0 / 1000.0;
        _map["cm"]     = 1.0 / 100.0;
        _map["m"]      = 1;
        _map["inch"]   = 0.0254;

        _map["deg"] = Deg2Rad;
        _map["rad"] = 1;

        _map["m/s"]  = 1;
        _map["cm/s"] = 1.0 / 100.0;
        _map["mm/s"] = 1.0 / 1000.0;

        _map["m/s^2"]  = 1;
        _map["cm/s^2"] = 1.0 / 100.0;
        _map["mm/s^2"] = 1.0 / 1000.0;

        _map["deg/s"] = Deg2Rad;
        _map["rad/s"] = 1;

        _map["deg/s^2"] = Deg2Rad;
        _map["rad/s^2"] = 1;
    };
    UnitMap (const std::map< std::string, double >& map) : _map (map) {}

    ~UnitMap () {}
};
}    // namespace

// const DOMCoreBasisTypes::UnitMap DOMCoreBasisTypes::_Units;
const UnitMap _Units;

double DOMCoreBasisTypes::getUnit (const std::string key)
{
    std::map< std::string, double >::const_iterator it = _Units._map.find (key);
    if (it == _Units._map.end ())
        RW_THROW ("Invalid Unit Attribute " << key);
    return (*it).second;
}

namespace {
double readUnit (DOMElem::Ptr element)
{
    std::string attrval = element->getAttributeValue (DOMCoreBasisTypes::idUnitAttribute (), "");
    if (!attrval.empty ())
        return DOMCoreBasisTypes::getUnit (attrval);
    return 1;
}

void checkHeader (DOMElem::Ptr element, const std::string id)
{
    if (!element->isName (id))
        RW_THROW ("Expected \"" << id << "\" got " << element->getName ());
}

template< class T >
inline T readVectorStructure (DOMElem::Ptr element, bool doCheckHeader, const std::string id)
{
    if (doCheckHeader)
        checkHeader (element, id);

    double scale                   = readUnit (element);
    std::vector< double > elements = element->getValueAsDoubleList ();

    T result;
    if (elements.size () != result.size ())
        RW_THROW ("Parse error: in element \"" << element->getName ()
                                               << "\" nr of elements must be \"" << result.size ()
                                               << "\" got \"" << elements.size () << "\"");

    for (size_t i = 0; i < elements.size (); i++) {
        result (i) = scale * elements[i];
    }
    return result;
}

}    // namespace

std::vector< double > DOMCoreBasisTypes::readDoubleList (DOMElem::Ptr element, bool doCheckHeader)
{
    return element->getValueAsDoubleList ();
}

std::vector< int > DOMCoreBasisTypes::readIntList (DOMElem::Ptr element, bool doCheckHeader)
{
    std::vector< double > res2 = element->getValueAsDoubleList ();
    std::vector< int > res (res2.size ());
    for (size_t i = 0; i < res2.size (); i++)
        res[i] = (int) res2[i];
    return res;
}

std::string DOMCoreBasisTypes::readString (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idString ());

    return element->getValue ();
}

std::vector< std::string > DOMCoreBasisTypes::readStringList (DOMElem::Ptr element)
{
    return element->getValueAsStringList ();
}

// typedef std::pair<std::string,std::string> StringPair;

StringPair DOMCoreBasisTypes::readStringPair (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idStringPair ());

    const std::vector< std::string > result = readStringList (element);

    if (result.size () != 2)
        RW_THROW ("Expected 2 string in StringPair but found" << result.size ());

    return std::make_pair (result[0], result[1]);
}

std::vector< StringPair > DOMCoreBasisTypes::readStringPairs (DOMElem::Ptr element)
{
    std::vector< StringPair > result;
    for (DOMElem::Ptr child : element->getChildren ()) {
        if (child->isName (idStringPair ())) {
            std::string str                    = readString (child);
            std::vector< std::string > strings = StringUtil::words (str);
            if (strings.size () != 2)
                RW_THROW ("Expected two string elements found " << strings.size () << " in \""
                                                                << str << "\"");
            result.push_back (std::make_pair (strings[0], strings[1]));
        }
    }
    return result;
}

double DOMCoreBasisTypes::readDouble (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idDouble ());
    return element->getValueAsDouble ();
}

float DOMCoreBasisTypes::readFloat (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idFloat ());
    return (float) element->getValueAsDouble ();
}

int DOMCoreBasisTypes::readInt (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idInteger ());

    return element->getValueAsInt ();
}

bool DOMCoreBasisTypes::readBool (DOMElem::Ptr element, bool doCheckHeader)
{
    if (doCheckHeader)
        checkHeader (element, idBoolean ());

    std::string str = element->getValue ();
    bool res        = false;
    try {
        res = boost::lexical_cast< bool > (str);
    }
    catch (...) {
        if (str == "true") {
            return true;
        }
        else if (str == "false") {
            return false;
        }
        else {
            RW_THROW ("Parse error: Could not parse bool, expected true,false,1 or 0 got \""
                      << str << "\"");
        }
    }
    return res;
}

namespace {
template< class T > std::string createStringFromArray (const T& v, size_t n)
{
    std::ostringstream str;
    str.unsetf (std::ios::floatfield);    // floatfield not set
    str.precision (16);
    for (size_t i = 0; i < n; ++i) {
        str << v (i);
        if (i != n - 1)
            str << " ";
    }
    return str.str ();
}

template< class T > std::string createStringFromArray (const T& v)
{
    return createStringFromArray< T > (v, v.size ());
}

template< class T > std::string createStringFromVector (const T& v, size_t n)
{
    std::ostringstream str;
    str.unsetf (std::ios::floatfield);    // floatfield not set
    str.precision (16);
    for (size_t i = 0; i < n; ++i) {
        str << v[i];
        if (i != n - 1)
            str << " ";
    }
    return std::string (str.str ());
}

template< class T > std::string createStringFromVector (const T& v)
{
    return createStringFromVector< T > (v, v.size ());
}
}    // namespace

DOMElem::Ptr DOMCoreBasisTypes::write (int val, DOMElem::Ptr elem, bool addHeader)
{
    if (addHeader)
        elem->setName (idInteger ());

    std::stringstream sstr;
    sstr << val;

    elem->setValue (sstr.str ());
    return elem;
}

DOMElem::Ptr DOMCoreBasisTypes::write (double val, DOMElem::Ptr elem, bool addHeader)
{
    if (addHeader)
        elem->setName (idDouble ());

    std::stringstream sstr;
    sstr << val;

    elem->setValue (sstr.str ());
    return elem;
}

DOMElem::Ptr DOMCoreBasisTypes::write (const std::string& str, DOMElem::Ptr elem, bool addHeader)
{
    if (addHeader)
        elem->setName (idString ());

    elem->setValue (str);
    return elem;
}

DOMElem::Ptr DOMCoreBasisTypes::createElement (const std::string& id, const std::string& value,
                                               DOMElem::Ptr doc)
{
    DOMElem::Ptr element = doc->addChild (id);
    element->setValue (value);
    return element;
}

DOMElem::Ptr DOMCoreBasisTypes::createIntList (const std::vector< int >& ints, DOMElem::Ptr doc)
{
    return createElement (idIntList (), createStringFromVector (ints), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createDoubleList (const std::vector< double >& doubles,
                                                  DOMElem::Ptr doc)
{
    return createElement (idDoubleList (), createStringFromVector (doubles), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createBoolean (bool value, DOMElem::Ptr doc)
{
    return createElement (idBoolean (), boost::lexical_cast< std::string > (value), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createDouble (double value, DOMElem::Ptr doc)
{
    return createElement (idDouble (), boost::lexical_cast< std::string > (value), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createFloat (float value, DOMElem::Ptr doc)
{
    return createElement (idFloat (), boost::lexical_cast< std::string > (value), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createInteger (int value, DOMElem::Ptr doc)
{
    return createElement (idInteger (), boost::lexical_cast< std::string > (value), doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createString (const std::string& str, DOMElem::Ptr doc)
{
    return createElement (idString (), str, doc);
}

DOMElem::Ptr DOMCoreBasisTypes::createStringList (const std::vector< std::string >& strings,
                                                  DOMElem::Ptr doc)
{
    DOMElem::Ptr element = doc->addChild (idStringList ());
    std::stringstream sstr;
    if (strings.size () > 0) {
        sstr << strings[0];
        for (size_t i = 1; i < strings.size (); i++) {
            sstr << ";" << strings[i];
        }
    }
    element->setValue (sstr.str ());
    return element;
}

DOMElem::Ptr DOMCoreBasisTypes::createStringPair (const std::string& first,
                                                  const std::string& second, DOMElem::Ptr doc)
{
    DOMElem::Ptr element = doc->addChild (idStringPair ());
    std::stringstream sstr;
    sstr << first << ";" << second;
    element->setValue (sstr.str ());
    return element;
}
