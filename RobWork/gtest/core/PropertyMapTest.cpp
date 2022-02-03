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

#include <rw/core/PropertyMap.hpp>

#include <functional>

using namespace rw::core;

namespace {
    void listener(int* cntId, int* cntOther, std::string id, PropertyMap* map, PropertyBase* prop)
    {
        if (prop->getIdentifier() == id)
            (*cntId)++;
        else
            (*cntOther)++;
    }
}

TEST(PropertyMapCore, PropertyMapCore) {
    PropertyMap map("name");
    EXPECT_EQ("name", map.getName());
    EXPECT_EQ(0, map.size());
    EXPECT_TRUE(map.empty());

    map.add("doubleId", "doubleDesc", 0.1);
    EXPECT_TRUE(map.has("doubleId"));
    EXPECT_EQ("doubleDesc", map.findPropertyBase("doubleId")->getDescription());
    EXPECT_EQ("doubleDesc", map.findProperty<double>("doubleId")->getDescription());
    EXPECT_EQ(0.1, map.findProperty<double>("doubleId")->getValue());
    EXPECT_EQ(0.1, map.get<double>("doubleId"));

    EXPECT_EQ(nullptr, map.findPropertyBase("nonExistingId"));
    EXPECT_EQ(nullptr, map.findProperty<double>("nonExistingId"));
    EXPECT_ANY_THROW(map.get<double>("nonExistingId"));
    EXPECT_EQ(1, map.size());
    EXPECT_FALSE(map.has("nonExistingId"));
    EXPECT_EQ(1.1, map.get<double>("nonExistingId", 1.1));
    EXPECT_EQ(2, map.size());
    EXPECT_TRUE(map.has("nonExistingId"));
    EXPECT_EQ(1.1, map.get<double>("nonExistingId"));

    map.add("doubleId", "NoChangeTest", 0.2);
    EXPECT_EQ("doubleDesc", map.findPropertyBase("doubleId")->getDescription());
    EXPECT_EQ("doubleDesc", map.findProperty<double>("doubleId")->getDescription());
    EXPECT_EQ(0.1, map.findProperty<double>("doubleId")->getValue());
    EXPECT_EQ(0.1, map.get<double>("doubleId"));

    map.addForce("doubleId", "OverwriteTest", 0.3);
    EXPECT_EQ("OverwriteTest", map.findPropertyBase("doubleId")->getDescription());
    EXPECT_EQ("OverwriteTest", map.findProperty<double>("doubleId")->getDescription());
    EXPECT_EQ(0.3, map.findProperty<double>("doubleId")->getValue());
    EXPECT_EQ(0.3, map.get<double>("doubleId"));

    map.add(ownedPtr(new Property<double>("doubleId", "NoChangeTest2", 0.4)));
    EXPECT_EQ("OverwriteTest", map.findPropertyBase("doubleId")->getDescription());
    EXPECT_EQ("OverwriteTest", map.findProperty<double>("doubleId")->getDescription());
    EXPECT_EQ(0.3, map.findProperty<double>("doubleId")->getValue());
    EXPECT_EQ(0.3, map.get<double>("doubleId"));

    map.set("doubleId", 0.5);
    EXPECT_EQ("OverwriteTest", map.findPropertyBase("doubleId")->getDescription());
    EXPECT_EQ("OverwriteTest", map.findProperty<double>("doubleId")->getDescription());
    EXPECT_EQ(0.5, map.findProperty<double>("doubleId")->getValue());
    EXPECT_EQ(0.5, map.get<double>("doubleId"));

    map.set("doubleId2", 0.6);
    EXPECT_EQ("", map.findPropertyBase("doubleId2")->getDescription());
    EXPECT_EQ("", map.findProperty<double>("doubleId2")->getDescription());
    EXPECT_EQ(0.6, map.findProperty<double>("doubleId2")->getValue());
    EXPECT_EQ(0.6, map.get<double>("doubleId2"));

    EXPECT_EQ(3, map.size());
    EXPECT_FALSE(map.empty());

    PropertyMap mapCopy = map;
    EXPECT_FALSE(mapCopy.empty());
    EXPECT_EQ("name", mapCopy.getName());
    EXPECT_EQ(0.5, *mapCopy.getPtr<double>("doubleId"));
    EXPECT_EQ(nullptr, mapCopy.getPtr<double>("DoesNotExist"));
    EXPECT_EQ(3, mapCopy.size());

    PropertyMap mapCopy2;
    mapCopy2 = map;
    EXPECT_FALSE(mapCopy2.empty());
    EXPECT_EQ("name", mapCopy2.getName());
    EXPECT_EQ(0.5, *mapCopy2.getPtr<double>("doubleId"));
    EXPECT_EQ(nullptr, mapCopy2.getPtr<double>("DoesNotExist"));
    EXPECT_EQ(3, mapCopy2.size());

    map.clear();
    EXPECT_TRUE(map.empty());
    EXPECT_EQ(0, map.size());
    EXPECT_FALSE(mapCopy.empty());
    EXPECT_EQ(3, mapCopy.size());
    EXPECT_FALSE(mapCopy2.empty());
    EXPECT_EQ(3, mapCopy2.size());

    mapCopy.erase("doubleId2");
    EXPECT_EQ(2, mapCopy.size());

    std::set<std::string> ids;
    for (const PropertyBase::Ptr& p : mapCopy.getProperties()) {
        ids.insert(p->getIdentifier());
    }
    EXPECT_TRUE(ids.find("doubleId") != ids.end());
    EXPECT_TRUE(ids.find("nonExistingId") != ids.end());
    EXPECT_EQ(2, ids.size());
}

TEST(PropertyMapCore, PropertyChangedListener) {
    using namespace std::placeholders;
    int cntId = 0;
    int cntOther = 0;
    PropertyMap map;
    map.addChangedListener(std::bind(&listener, &cntId, &cntOther, "Id", _1, _2));
    EXPECT_EQ(0, cntId);
    EXPECT_EQ(0, cntOther);
    map.add("Id", "Desc", 0.1);
    EXPECT_EQ(1, cntId);
    EXPECT_EQ(0, cntOther);
    map.notifyListeners(map.findPropertyBase("Id").get());
    EXPECT_EQ(2, cntId);
    EXPECT_EQ(0, cntOther);
    map.set("Id", 0.2);
    EXPECT_EQ(3, cntId);
    EXPECT_EQ(0, cntOther);

    Property<double>::Ptr prop = map.findProperty<double>("Id");
    map.erase("Id");
    prop->setValue(0.3);
    EXPECT_EQ(3, cntId);
    EXPECT_EQ(0, cntOther);

    map.add("Other", "Desc", 0.1);
    EXPECT_EQ(3, cntId);
    EXPECT_EQ(1, cntOther);
    Property<double>::Ptr prop2 = map.findProperty<double>("Other");
    map.clear();
    prop2->setValue(0.3);
    EXPECT_EQ(3, cntId);
    EXPECT_EQ(1, cntOther);
}

TEST(PropertyMapCore, swap) {
    using namespace std::placeholders;
    int cntIdA = 0;
    int cntOtherA = 0;
    int cntIdB = 0;
    int cntOtherB = 0;
    PropertyMap mapA;
    mapA.addChangedListener(std::bind(&listener, &cntIdA, &cntOtherA, "IdA1", _1, _2));
    PropertyMap mapB;
    mapB.addChangedListener(std::bind(&listener, &cntIdB, &cntOtherB, "IdB1", _1, _2));

    mapA.add("IdA1", "DescA1", 0.1);
    mapA.add("IdA2", "DescA2", 0.2);
    mapB.add("IdB1", "DescB1", 0.3);
    mapB.add("IdB2", "DescB2", 0.4);

    EXPECT_EQ(1, cntIdA);
    EXPECT_EQ(1, cntOtherA);
    EXPECT_EQ(1, cntIdB);
    EXPECT_EQ(1, cntOtherB);

    mapA.swap(mapB);
    EXPECT_EQ(2, mapA.size());
    EXPECT_EQ(2, mapB.size());
    EXPECT_TRUE(mapA.has("IdB1"));
    EXPECT_TRUE(mapA.has("IdB2"));
    EXPECT_TRUE(mapB.has("IdA1"));
    EXPECT_TRUE(mapB.has("IdA2"));

    EXPECT_EQ(1, cntIdA);
    EXPECT_EQ(3, cntOtherA);
    EXPECT_EQ(1, cntIdB);
    EXPECT_EQ(3, cntOtherB);
}
