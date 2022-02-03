/******************************************************************************
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
 ******************************************************************************/

#include <gtest/gtest.h>

#include "../TestEnvironment.hpp"

#include <rw/core/StringUtil.hpp>

using rw::core::StringUtil;

TEST(StringUtil, GetRelativeDirectoryNameTest_HardcodedPathTest) {
    std::string wcPath = TestEnvironment::testfilesDir() + "workcells/simple_wc/SimpleWorkcell.wc.xml";
    const std::string robotGeometryTestString = TestEnvironment::testfilesDir() + "workcells/simple_wc/PA10/Geometry/Rob-0";
    const std::string hardcodedResult = "PA10/Geometry/";
    const std::string result = StringUtil::getRelativeDirectoryName(robotGeometryTestString, StringUtil::getDirectoryName(wcPath));
    EXPECT_EQ(hardcodedResult, result);
}
