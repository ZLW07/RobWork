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

#include "ScriptTypes.hpp"

using rw::core::ownedPtr;

namespace {
::rw::core::LogWriter::Ptr _writer;
}

void rwlibs::swig::writelog (const std::string& msg)
{
    ::rw::core::Log::log ().setEnable (::rw::core::Log::User8Mask);
    if (_writer.isNull ())
        rwlibs::swig::setlog (&rw::core::Log::infoLog ());
    ::rw::core::Log::log ().write (::rw::core::Log::User8, msg);
}

void rwlibs::swig::setlog (::rw::core::LogWriter::Ptr writer)
{
    _writer = writer;
    ::rw::core::Log::log ().setWriter (::rw::core::Log::User8, writer);
}
/*
rw::math::Rotation3D<> rwlibs::swig::getRandomRotation3D ()
{
    return ::rw::math::Math::ranRotation3D< double > ();
}

rw::math::Transform3D< double > rwlibs::swig::getRandomTransform3D (const double translationLength)
{
    return ::rw::math::Math::ranTransform3D< double > (translationLength);
}
*/