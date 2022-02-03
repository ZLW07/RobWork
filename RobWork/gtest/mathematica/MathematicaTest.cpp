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

#include <gtest/gtest.h>

#include <rw/common/Exception.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/sensor/Image.hpp>
#include <rwlibs/mathematica.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

TEST(MathematicaTest, MathematicaTest ){

	try {
		Mathematica m;
		EXPECT_TRUE(m.initialize());
		const Mathematica::Link::Ptr l = m.launchKernel();
		EXPECT_FALSE(l.isNull());
		EXPECT_TRUE(l->isOpen());
		Mathematica::Packet::Ptr result;
		*l >> result; // read the first In[1]:= prompt from kernel
		EXPECT_EQ(result->packetType() , Mathematica::InputName);

		{
			// Solve problem that is well-defined
			const char* const cmd = "Solve[x^2 + 2 y^3 == 3681 && x > 0 && y > 0, {x, y}, Integers]";
			*l << cmd;
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::Return);
			const ReturnPacket::Ptr packet = result.cast<ReturnPacket>();
			EXPECT_FALSE(packet.isNull());
			try {
				const List::Ptr list = List::fromExpression(*packet->expression());
				EXPECT_FALSE(list.isNull());
				const std::list<rw::common::Ptr<const Mathematica::Expression> > sols = list->getArguments();
				EXPECT_EQ(sols.size() , 3);
				for(const rw::common::Ptr<const Mathematica::Expression> sol : sols) {
					const rw::common::Ptr<const Mathematica::FunctionBase> fct = sol.cast<const Mathematica::FunctionBase>();
					const PropertyMap::Ptr map = Rule::toPropertyMap(fct->getArguments());
					const int x = map->get<int>("x");
					const int y = map->get<int>("y");
					EXPECT_EQ(x*x+2*y*y*y , 3681);
					EXPECT_GT(x , 0);
					EXPECT_GT(y , 0);
				}
			} catch (const Exception& e) {
				std::cout << e.what() << std::endl;
				EXPECT_FALSE(true);
			}
		}

		{
			// Solve problem that gives a warning (using full evaluation mode)
			const char* const cmd = "Solve[Sin[x] == 0.5, x]";
			*l << EnterTextPacket(cmd);
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::Message);
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::Text);
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::OutputName);
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::ReturnText);
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::InputName);
		}

		{
			// Construct data for a ListPlot
			std::vector<std::string> axesLabel(2);
			axesLabel[0] = "X axis";
			axesLabel[1] = "Y axis";
			PropertyMap properties;
			properties.add<std::string>("PlotLabel","","TESTPLOT");
			properties.add("AxesLabel","",axesLabel);
			std::vector<double> x,y;
			for (std::size_t i = 0; i < 100; i++) {
				x.push_back(i);
				y.push_back(i*i);
			}
			std::list<Mathematica::Expression::Ptr> options;
			for(const Rule::Ptr option : Rule::toRules(properties)) {
				options.push_back(option);
			}
			List imageSize;
			imageSize.add(400).add(250);
			options.push_back(ownedPtr(new Rule("ImageSize",imageSize)));

			long long start;
			long long end;
			// Send the command
			*l << Image(ListPlot(x,y,options));
			start = TimerUtil::currentTimeMs();
			l->wait();
			end = TimerUtil::currentTimeMs();
			start = TimerUtil::currentTimeMs();
			*l >> result;
			end = TimerUtil::currentTimeMs();
			EXPECT_EQ(result->packetType() , Mathematica::Return);
			const ReturnPacket::Ptr imgPacket = result.cast<ReturnPacket>();
			EXPECT_FALSE(imgPacket.isNull());

			// Now convert to RW image
			start = TimerUtil::currentTimeMs();
			const rw::sensor::Image::Ptr image = Image::toRobWorkImage(*imgPacket->expression());
			end = TimerUtil::currentTimeMs();
			EXPECT_FALSE(image.isNull());
			image->saveAsPPM("test.ppm");
		}
	} catch(const Exception& e) {
			std::cout << e.what() << std::endl;
			EXPECT_FALSE(true);
	}
}

#if __cplusplus >= 201103L
TEST(MathematicaTest, MathematicaTestCPP11 ){
	try {
		Mathematica m;
		EXPECT_TRUE(m.initialize());
		const Mathematica::Link::Ptr l = m.launchKernel();
		EXPECT_FALSE(l.isNull());
		EXPECT_TRUE(l->isOpen());
		Mathematica::Packet::Ptr result;
		*l >> result; // read the first In[1]:= prompt from kernel

		{
			*l << Image(ListPlot({{0,1},{2,2},{2.5,3}},Rule("PlotLabel","TESTPLOT C++11"),Rule("AxesLabel",{"X","Y"})));
			*l >> result;
			EXPECT_EQ(result->packetType() , Mathematica::Return);
			const ReturnPacket::Ptr imgPacket = result.cast<ReturnPacket>();
			EXPECT_FALSE(imgPacket.isNull());

			// Now convert to RW image
			const rw::sensor::Image::Ptr image = Image::toRobWorkImage(*imgPacket->expression());
			EXPECT_FALSE(image.isNull());
			image->saveAsPPM("test11.ppm");
		}
	} catch(const Exception& e) {
			std::cout << e.what() << std::endl;
			EXPECT_FALSE(true);	}
}
#endif
