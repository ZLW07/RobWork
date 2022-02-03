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

#include "Q.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::core;
using namespace rw::math;

template<> Q::Q (size_t n, const double* values) : _vec (n)
{
    for (size_t i = 0; i < n; i++) {
        _vec (i) = values[i];
    }
}

template<> Q::Q (size_t n, double* values) : _vec (n)
{
    for (size_t i = 0; i < n; i++) {
        _vec (i) = values[i];
    }
}

template<> Q::Q (int n, const double* values) : _vec (n)
{
    for (int i = 0; i < n; i++) {
        _vec (i) = values[i];
    }
}

template<> Q::Q (int n, double* values) : _vec (n)
{
    for (int i = 0; i < n; i++) {
        _vec (i) = values[i];
    }
}

template<> Q::Q (int n, double value) : _vec (n)
{
    for (int i = 0; i < n; i++) {
        _vec (i) = value;
    }
}

template<> Q::Q (size_t n, double value) : _vec (n)
{
    for (size_t i = 0; i < n; i++) {
        _vec (i) = value;
    }
}

Q::Q (const std::vector< double >& r) : _vec (r.size ())
{
    for (size_t i = 0; i < r.size (); i++)
        _vec (i) = r[i];
}

Q::~Q ()
{}

std::ostream& rw::math::operator<< (std::ostream& out, const Q& v)
{
    if (v.size () == 0) {
        return out << "Q[0]{}";
    }
    else {
        out << "Q[" << (int) v.size () << "]{";
        for (size_t i = 0; i < v.size () - 1; i++)
            out << v[i] << ", ";
        return out << v[v.size () - 1] << "}";
    }
}

std::istream& rw::math::operator>> (std::istream& in, Q& q)
{
    char ch1, ch2;
    do {
        in.get (ch1);
    } while (ch1 == ' ' || ch1 == '\t');    // Ignore space and tab, but not line changes.

    int size = -1;

    if (ch1 == 'Q') {
        in.get (ch2);
        if (ch1 != 'Q' || ch2 != '[')
            RW_THROW ("Content of input stream does not match format of Q");
        in >> size;

        in.get (ch1);
        in.get (ch2);
        if (ch1 != ']' || ch2 != '{')
            RW_THROW ("Content of input stream does not match format of Q");
    }
    else if (ch1 != '{') {
        RW_THROW ("Content of input stream does not match format of Q");
    }

    std::vector< double > res;
    while (ch1 != '}') {
        double d;
        in >> d;
        if (!in.eof ()) {
            res.push_back (d);
        }
        in.get (ch1);
    }

    if (ch1 != '}')
        RW_THROW ("Content of input stream does not match format of Q");

    if (size > -1 && (int) res.size () != size) {
        RW_THROW ("Length of Q does not match device");
    }

    q = Q (res.size (), &res[0]);
    return in;
}

bool rw::math::Q::operator== (const Q& q2) const
{
    if (this->size () != q2.size ())
        return false;

    for (size_t i = 0; i < this->size (); i++)
        if ((*this) (i) != q2 (i))
            return false;
    return true;
}

double rw::math::dot (const Q& a, const Q& b)
{
    return a.e ().dot (b.e ());
    // return inner_prod(a.m(), b.m());
}

rw::math::Q rw::math::concat (const Q& q1, const Q& q2)
{
    Q q (q1.size () + q2.size ());
    for (size_t i = 0; i < q1.size (); i++)
        q (i) = q1 (i);
    for (size_t i = 0; i < q2.size (); i++)
        q (q1.size () + i) = q2 (i);
    return q;
}

template<>
void rw::common::serialization::write (const Q& tmp, OutputArchive& oar, const std::string& id)
{
    oar.write (tmp.toStdVector (), id, "Q");
}
template<> void rw::common::serialization::read (Q& tmp, InputArchive& iar, const std::string& id)
{
    std::vector< double > arr;
    iar.read (arr, id, "Q");
    tmp = Q (arr);
    // rw::math::Math::fromStdVector(arr, tmp);
}

template<>
rw::core::Ptr< Property< Q > > PropertyMap::findProperty (const std::string& identifier) const
{
    std::string id       = identifier;
    PropertyBase::Ptr pb = findPropertyBase (id);
    if (!pb.isNull () && pb->getType ().getId () == PropertyType::DoubleList) {
        id = id + "<Q>";
        if (findPropertyBase (id).isNull ()) {
            std::vector< double > q = findPropertyBase (identifier)
                                          .cast< Property< std::vector< double > > > ()
                                          ->getValue ();

            rw::core::Ptr< Property< Q > > prop = ownedPtr (
                new Property< Q > (pb->getIdentifier () + "<Q>", pb->getDescription (), Q (q)));

            const_cast< PropertyMap* > (this)->add (prop);
        }
    }
    return findPropertyBase (id).cast< Property< rw::math::Q > > ();
    ;
}
