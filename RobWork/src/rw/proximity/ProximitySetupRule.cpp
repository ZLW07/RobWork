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

#include "ProximitySetupRule.hpp"

#include <rw/core/StringUtil.hpp>

#include <regex>

using namespace rw::core;
using namespace rw::proximity;

namespace {
std::regex convertToRegEx (const std::string& ex)
{
    if (ex.size () > 5 && ex.substr (0, 5) == "REGEX") {
        return std::regex (ex.substr (5, ex.size () - 5));
    }
    else {
        return std::regex (StringUtil::patternToRegEx (ex));
    }
}
}    // namespace

ProximitySetupRule::ProximitySetupRule():
    _type(ProximitySetupRule::EXCLUDE_RULE)
{
}

ProximitySetupRule::ProximitySetupRule (const std::string& patternA, const std::string& patternB,
                                        RuleType type) :
    _patterns (patternA, patternB),
    _regex1 (convertToRegEx (patternA)), _regex2 (convertToRegEx (patternB)), _type (type)
{}
