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

#include "DOMElem.hpp"

#include <rw/core/StringUtil.hpp>

#include <boost/lexical_cast.hpp>
#include <string>

using namespace rw::core;

bool DOMElem::getValueAsBool ()
{
    std::string val  = getValue ();
    std::string valu = rw::core::StringUtil::toUpper (val);
    if (valu == "TRUE")
        return true;
    if (valu == "FALSE")
        return false;
    return boost::lexical_cast< bool > (val);
}

void DOMElem::setValue (bool val)
{
    setValue (boost::lexical_cast< std::string > (val));
}

void DOMElem::setValue (int val)
{
    setValue (boost::lexical_cast< std::string > (val));
}

void DOMElem::setValue (double val)
{
    setValue (boost::lexical_cast< std::string > (val));
}

void DOMElem::setValueString (std::string val)
{
    setValue (val);
}

std::string DOMElem::getAttributeValue (const std::string& name, const std::string& default_value)
{
    if (DOMElem::Ptr attrib = getAttribute (name, true))
        return attrib->getValue ();
    return default_value;
}

bool DOMElem::getAttributeValueAsBool (const std::string& name, bool default_value)
{
    if (DOMElem::Ptr attrib = getAttribute (name, true))
        return attrib->getValueAsBool ();
    return default_value;
}

double DOMElem::getAttributeValueAsDouble (const std::string& name, double default_value)
{
    if (DOMElem::Ptr attrib = getAttribute (name, true))
        return attrib->getValueAsDouble ();
    return default_value;
}

int DOMElem::getAttributeValueAsInt (const std::string& name, int default_value)
{
    if (DOMElem::Ptr attrib = getAttribute (name, true))
        return attrib->getValueAsInt ();
    return default_value;
}