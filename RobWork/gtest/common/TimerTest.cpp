/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/common/Timer.hpp>

using namespace rw::common;

TEST(TimerTest, GetTimeFunctions) {
	Timer t1(3030);
	EXPECT_EQ(t1.getTimeMs(),3030);
	EXPECT_EQ(t1.getTimeSec(),3);
}

TEST(TimerTest, ToStringFunctions) {
	std::string tstr1 = Timer(1,2,30,10).toString("hh:mm:ss");
	EXPECT_EQ(tstr1,"01:02:30") << "Should be: " << tstr1;
	std::string tstr2 = Timer(1,2,2,10).toString("hh:mm");
	EXPECT_EQ(tstr2,"01:02") << "Should be: " << tstr2;
	std::string tstr3 = Timer(1,2,30,10).toString("h:m:s");
	EXPECT_EQ(tstr3,"1:2:30") << "Should be: " << tstr3;
	std::string tstr4 = Timer(1,2,2,10).toString("h:m");
	EXPECT_EQ(tstr4,"1:2") << "Should be: " << tstr4;
}

