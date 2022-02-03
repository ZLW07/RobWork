/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/common/ProgramOptions.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/PropertyBase.hpp>
#include <boost/program_options.hpp>
#include <rw/math/Q.hpp>

#include <string>

using namespace rw::common;
using namespace rw::core;
using boost::program_options::options_description;
using boost::program_options::option_description;
using boost::program_options::positional_options_description; 

TEST(ProgramOptions, Constructor) {
    ProgramOptions parser("testApp","6.6.6");
    PropertyMap p = parser.getPropertyMap();

    options_description od = parser.getOptionDescription();
    EXPECT_EQ(od.options().size(),0u);

    positional_options_description pod = parser.getPosOptionDescription();
    EXPECT_EQ(pod.max_total_count(),0u);

    EXPECT_EQ(p.getName(),"");
    EXPECT_EQ(p.getProperties().begin(),p.getProperties().end());
}

TEST(ProgramOptions, initOptions) {
    ProgramOptions parser("testApp","6.6.6");
    parser.initOptions();

    options_description od = parser.getOptionDescription();
    EXPECT_EQ(od.options().size(),6u);

    positional_options_description pod = parser.getPosOptionDescription();
    EXPECT_EQ(pod.max_total_count(),0u);

    PropertyMap p = parser.getPropertyMap();

    EXPECT_EQ(p.getName(),"");
    EXPECT_EQ(p.getProperties().begin(),p.getProperties().end());
}

TEST(ProgramOptions, ParseInitOptions) {
    ProgramOptions parser("testApp","6.6.6");
    parser.initOptions();

    rw::common::Log::getInstance()->setLevel(Log::LogIndex::Warning);
    EXPECT_EQ(parser.parse("--help"),-1);
    EXPECT_EQ(parser.parse("--version"),-1);

    std::string args = "-isize=3 -dfix=2.3 -PSomething=HalloWorld -qList=(221.2342,2.0,3,4)";
    parser.parse(args);
    PropertyMap p = parser.getPropertyMap();

    int size = p.get<int>("size");
    double fix = p.get<double>("fix");
    std::string somthing = p.get<std::string>("Something");
    rw::math::Q List = p.get<rw::math::Q>("List");

    EXPECT_EQ(size,3);
    EXPECT_EQ(fix,2.3);
    EXPECT_EQ(somthing, "HalloWorld");
    EXPECT_EQ(List,rw::math::Q(4,221.2342,2.0,3.0,4.0));

    args = "--intproperty size=3 --doubleproperty fix=2.3 --property Something=HalloWorld --qproperty List=(221.2342,2.0,3,4)";
    parser.parse(args);

    size = p.get<int>("size");
    fix = p.get<double>("fix");
    somthing = p.get<std::string>("Something");
    List = p.get<rw::math::Q>("List");

    EXPECT_EQ(size,3);
    EXPECT_EQ(fix,2.3);
    EXPECT_EQ(somthing, "HalloWorld");
    EXPECT_EQ(List,rw::math::Q(4,221.2342,2.0,3.0,4.0));
}

//TODO(kalor) add test of setPositionOption and addStringOption
