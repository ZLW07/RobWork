/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "EvaluatePacket.hpp"

using namespace rw::common;
using namespace rwlibs::mathematica;

EvaluatePacket::EvaluatePacket (const Mathematica::Expression& expression) :
    Packet ("EvaluatePacket", Mathematica::Evaluate), _expression (expression.clone ())
{}

EvaluatePacket::EvaluatePacket (rw::core::Ptr< const Mathematica::Expression > expression) :
    Packet ("EvaluatePacket", Mathematica::Evaluate), _expression (expression)
{}

EvaluatePacket::~EvaluatePacket ()
{}

const rw::core::Ptr< const Mathematica::Expression > EvaluatePacket::expression ()
{
    return _expression;
}

std::list< rw::core::Ptr< const Mathematica::Expression > > EvaluatePacket::getArguments () const
{
    return std::list< rw::core::Ptr< const Mathematica::Expression > > (1, _expression);
}

Mathematica::Expression::Ptr EvaluatePacket::clone () const
{
    return ownedPtr (new EvaluatePacket (*_expression));
}
