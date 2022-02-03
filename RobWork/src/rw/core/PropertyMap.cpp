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

#include "PropertyMap.hpp"

#include <functional>

using namespace rw::core;

PropertyMap::PropertyMap ()
{}

PropertyMap::~PropertyMap ()
{
    _properties.clear ();
}

PropertyMap::PropertyMap (const PropertyMap& other):
    _name(other.getName())
{
    // Clone all property base objects.
    for (MapType::iterator it = other._properties.begin (); it != other._properties.end (); it++) {
        const PropertyBase::Ptr base = *it;
        this->insert (ownedPtr (base->clone ()));
    }
}

bool PropertyMap::add (PropertyBase::Ptr property)
{
    return insert (property);
}

PropertyMap& PropertyMap::operator= (const PropertyMap& other)
{
    // Assignment operator by the swap-idiom.
    if (this != &other) {
        PropertyMap copy = other;
        swap (copy);
        _name = other.getName();
    }
    return *this;
}

void PropertyMap::swap (PropertyMap& other)
{
    using std::placeholders::_1;
    _properties.swap (other._properties);
    for (const PropertyBase::Ptr& p : _properties) {
        p->changedEvent().remove(&other);
        p->changedEvent().add(std::bind(&PropertyMap::propertyChangedListener, this, _1), this );
        propertyChangedListener(p.get());
    }
    for (const PropertyBase::Ptr& p : other.getProperties()) {
        p->changedEvent().remove(this);
        p->changedEvent().add(std::bind(&PropertyMap::propertyChangedListener, &other, _1), &other );
        other.propertyChangedListener(p.get());
    }
}

bool PropertyMap::has (const std::string& identifier) const
{
    return findPropertyBase (identifier) != NULL;
}

bool PropertyMap::erase (const std::string& identifier)
{
    Property< int > key (identifier, "", 0);

    typedef MapType::iterator I;
    const I p = _properties.find (&key);
    if (p != _properties.end ()) {
        (*p)->changedEvent().remove(this);
        _properties.erase (p);
        return true;
    }
    else {
        return false;
    }
}

void PropertyMap::clear ()
{
    for (const PropertyBase::Ptr& p : _properties) {
        p->changedEvent().remove(this);
    }
    _properties.clear ();
}

size_t PropertyMap::size () const
{
    return _properties.size ();
}

bool PropertyMap::empty () const
{
    return _properties.empty ();
}

bool PropertyMap::insert (PropertyBase::Ptr property)
{
    using std::placeholders::_1;
    if (_properties.insert (property).second) {
        // add to changed listener
        property->changedEvent().add(
                std::bind(&PropertyMap::propertyChangedListener, this, _1),
                this
        );
        propertyChangedListener(property.get());
        return true;
    }
    return false;
}

PropertyBase::Ptr PropertyMap::findPropertyBase (const std::string& identifier)
{
    Property< int > key (identifier, "", 0);
    typedef MapType::iterator I;
    const I p = _properties.find (&key);
    if (p != _properties.end ())
        return *p;
    return NULL;
}

const PropertyBase::Ptr PropertyMap::findPropertyBase (const std::string& identifier) const
{
    return const_cast< PropertyMap* > (this)->findPropertyBase (identifier);
}

rw::core::iter_pair< PropertyMap::iterator > PropertyMap::getProperties () const
{
    return rw::core::iter_pair< PropertyMap::iterator > (
        std::make_pair (_properties.begin (), _properties.end ()));
}

void PropertyMap::notifyListeners (PropertyBase* base)
{
    for(const PropertyChangedListener& listener : _listeners) {
        listener(this, base);
    }
}

void PropertyMap::addChangedListener (PropertyChangedListener callback)
{
    _listeners.push_back (callback);
}

void PropertyMap::propertyChangedListener (PropertyBase* base)
{
    std::string id = base->getIdentifier ();
    // std::cout << "PropertyMap: Property Changed Listerner: " << id << std::endl;
    // notify all listeners
    notifyListeners (base);
}

void PropertyMap::clearChangedListeners (){
    _listeners.clear();
}
