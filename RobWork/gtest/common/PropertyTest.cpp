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

#include <gtest/gtest.h>
#include <rw/core/Property.hpp>
#include <rw/core/PropertyBase.hpp>
#include <rw/core/PropertyMap.hpp>

using namespace rw::core;

TEST(PropertyTest, Property)
{
    //Test basic functionality
    Property<double>::Ptr propA = ownedPtr(new Property<double>("A", "propA", 123.456));
    EXPECT_EQ(propA->getIdentifier() , "A");
    EXPECT_EQ(propA->getDescription() , "propA");
    EXPECT_EQ(propA->getValue() , 123.456);

    Property<std::string>::Ptr propB = ownedPtr( new Property<std::string>("B", "propB", "HELLO") );
    EXPECT_EQ(propB->getIdentifier() , "B");
    EXPECT_EQ(propB->getDescription() , "propB");
    EXPECT_EQ(propB->getValue() , "HELLO");

    PropertyBase *propPointer = propB.get();
    EXPECT_EQ(propPointer->getIdentifier() , "B");
    EXPECT_EQ(propPointer->getDescription() , "propB");

    PropertyMap bag;
    bag.add(
        propA->getIdentifier(),
        propA->getDescription(),
        propA->getValue());

    bag.add(
        propB->getIdentifier(),
        propB->getDescription(),
        propB->getValue());

    EXPECT_EQ(bag.size() , 2);

    for(PropertyBase::Ptr prop: bag.getProperties()) {
        std::string str = prop->getIdentifier();
        EXPECT_NE(str , "");
    }

    PropertyBase::Ptr p = bag.findPropertyBase("B");
    EXPECT_FALSE(p.isNull());
    EXPECT_EQ(p->getDescription() , "propB");

    Property<double>::Ptr pd = bag.findProperty<double>("A");
    EXPECT_FALSE(pd.isNull());
    EXPECT_EQ(pd->getValue() , 123.456);

    // Test that NULL is returned if types do not match
    pd = bag.findProperty<double>("B");
    EXPECT_TRUE(pd.isNull());
}
