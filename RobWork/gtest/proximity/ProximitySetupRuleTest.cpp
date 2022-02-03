/********************************************************************************
 * Copyright 2020 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/proximity/ProximitySetupRule.hpp>

#include <gtest/gtest.h>

using rw::proximity::ProximitySetupRule;

TEST (ProximitySetupRule, ProximitySetupRule)
{
    {
        const ProximitySetupRule rule("A*", "B", ProximitySetupRule::INCLUDE_RULE);
        EXPECT_EQ(ProximitySetupRule::INCLUDE_RULE, rule.type());
        EXPECT_EQ("A*", rule.getPatterns().first);
        EXPECT_EQ("B", rule.getPatterns().second);
        EXPECT_TRUE(rule.match("A1", "B"));
        EXPECT_FALSE(rule.match("A1", "Base"));
        EXPECT_FALSE(rule.match("a1", "B"));
        EXPECT_TRUE(rule.match("A", "B"));
        std::pair<std::string, std::string> pair("A1", "B");
        EXPECT_TRUE(rule.match(pair));

        const ProximitySetupRule rule2 = ProximitySetupRule::makeInclude("A*", "B");
        EXPECT_EQ(rule, rule2);
    }
    {
        const ProximitySetupRule rule = ProximitySetupRule::makeExclude("A*", "B");
        EXPECT_EQ("A*", rule.getPatterns().first);
        EXPECT_EQ("B", rule.getPatterns().second);
        EXPECT_TRUE(rule.match("A1", "B"));
        EXPECT_FALSE(rule.match("A1", "Base"));
        std::pair<std::string, std::string> pair("A1", "B");
        EXPECT_TRUE(rule.match(pair));
    }
    {
        const ProximitySetupRule rule("A*.Test", "B.*", ProximitySetupRule::INCLUDE_RULE);
        EXPECT_FALSE(rule.match("A1", "B"));
        EXPECT_FALSE(rule.match("A1", "B.Test"));
        EXPECT_TRUE(rule.match("A1.Test", "B.Test"));
        EXPECT_TRUE(rule.match("A.Test", "B.Test"));
        EXPECT_FALSE(rule.match("ATest", "B.Test"));
    }
}
