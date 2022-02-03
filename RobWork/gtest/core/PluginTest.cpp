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

#include "../TestEnvironment.hpp"

#include <rw/core/DOMParser.hpp>
#include <rw/core/Plugin.hpp>

using namespace rw::core;

TEST(PluginTest, loadDirectPlugin)
{
    const DOMParser::Ptr parser = DOMParser::make();
    parser->load(TestEnvironment::executableDir() + "test_plugin.rwplugin.xml");
    const DOMElem::Ptr elem = parser->getRootElement()->getChild("plugin");
    const DOMElem::Ptr runtime = elem->getChild("runtime");
    const std::string libpath = runtime->getAttributeValue("library");

    const rw::core::Ptr<Plugin> plugin = Plugin::load(libpath);

    EXPECT_EQ("TestPlugin", plugin->getId());
    EXPECT_EQ("Name of test plugin", plugin->getName());
    EXPECT_EQ("1.2.3", plugin->getVersion());

    const std::vector<std::string> ids = plugin->getExtensionPointIDs();
    EXPECT_EQ(3u, ids.size());
    if (ids.size() >= 3u) {
        EXPECT_EQ("ExtensionId1", ids[0]);
        EXPECT_EQ("ExtensionId2", ids[1]);
        EXPECT_EQ("ExtensionId3", ids[2]);
    }

    const std::vector<Extension::Descriptor> extDesc =
            plugin->getExtensionDescriptors();
    EXPECT_EQ(3u, extDesc.size());
    if (ids.size() >= 3u) {
        EXPECT_EQ("ExtensionId1", extDesc[0].id);
        EXPECT_EQ("ExtensionId2", extDesc[1].id);
        EXPECT_EQ("ExtensionId3", extDesc[2].id);
        EXPECT_EQ("", extDesc[0].name);
        EXPECT_EQ("", extDesc[1].name);
        EXPECT_EQ("", extDesc[2].name);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", extDesc[0].point);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", extDesc[1].point);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointB", extDesc[2].point);
    }
    EXPECT_TRUE(extDesc[0].props.has("TestProperty"));
    EXPECT_EQ("prop",
            extDesc[0].props.get("TestProperty", std::string("Not found!")));

    Extension::Ptr ext1 = plugin->makeExtension("ExtensionId1");
    Extension::Ptr ext2 = plugin->makeExtension("ExtensionId2");
    Extension::Ptr ext3 = plugin->makeExtension("ExtensionId3");
    EXPECT_FALSE(ext1.isNull());
    EXPECT_FALSE(ext2.isNull());
    EXPECT_FALSE(ext3.isNull());

    EXPECT_EQ("ExtensionId1", ext1->getId());
    EXPECT_EQ("ExtensionId2", ext2->getId());
    EXPECT_EQ("ExtensionId3", ext3->getId());
    EXPECT_EQ("", ext1->getName());
    EXPECT_EQ("", ext2->getName());
    EXPECT_EQ("", ext3->getName());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", ext1->getPoint());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", ext2->getPoint());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointB", ext3->getPoint());
    EXPECT_TRUE(ext1->getProperties().has("TestProperty"));
    EXPECT_EQ("prop",
            ext1->getProperties().get("TestProperty",
                    std::string("Not found!")));
}

TEST(PluginTest, loadLazyPlugin)
{
    std::string xmlpath = TestEnvironment::executableDir();
    xmlpath += "test_plugin.rwplugin.xml";
    const rw::core::Ptr<Plugin> plugin = Plugin::load(xmlpath);

    EXPECT_EQ("TestLazyPlugin", plugin->getId());
    EXPECT_EQ("Name of plugin for test.", plugin->getName());
    EXPECT_EQ("1.0", plugin->getVersion());

    const std::vector<std::string> ids = plugin->getExtensionPointIDs();
    EXPECT_EQ(3u, ids.size());
    if (ids.size() >= 3u) {
        EXPECT_EQ("ExtensionId1", ids[0]);
        EXPECT_EQ("ExtensionId2", ids[1]);
        EXPECT_EQ("ExtensionId3", ids[2]);
    }

    const std::vector<Extension::Descriptor> extDesc =
            plugin->getExtensionDescriptors();
    EXPECT_EQ(3u, extDesc.size());
    if (ids.size() >= 3u) {
        EXPECT_EQ("ExtensionId1", extDesc[0].id);
        EXPECT_EQ("ExtensionId2", extDesc[1].id);
        EXPECT_EQ("ExtensionId3", extDesc[2].id);
        EXPECT_EQ("Name of first extension.", extDesc[0].name);
        EXPECT_EQ("Name of second extension.", extDesc[1].name);
        EXPECT_EQ("Name of third extension.", extDesc[2].name);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", extDesc[0].point);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", extDesc[1].point);
        EXPECT_EQ("sdurw_core-gtest.ExtensionPointB", extDesc[2].point);
    }
    EXPECT_TRUE(extDesc[0].props.has("TestProperty"));
    EXPECT_EQ("prop",
            extDesc[0].props.get("TestProperty", std::string("Not found!")));

    Extension::Ptr ext1 = plugin->makeExtension("ExtensionId1");
    Extension::Ptr ext2 = plugin->makeExtension("ExtensionId2");
    Extension::Ptr ext3 = plugin->makeExtension("ExtensionId3");
    EXPECT_FALSE(ext1.isNull());
    EXPECT_FALSE(ext2.isNull());
    EXPECT_FALSE(ext3.isNull());

    EXPECT_EQ("ExtensionId1", ext1->getId());
    EXPECT_EQ("ExtensionId2", ext2->getId());
    EXPECT_EQ("ExtensionId3", ext3->getId());
    EXPECT_EQ("", ext1->getName());
    EXPECT_EQ("", ext2->getName());
    EXPECT_EQ("", ext3->getName());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", ext1->getPoint());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointA", ext2->getPoint());
    EXPECT_EQ("sdurw_core-gtest.ExtensionPointB", ext3->getPoint());
    EXPECT_TRUE(ext1->getProperties().has("TestProperty"));
    EXPECT_EQ("prop",
            ext1->getProperties().get("TestProperty",
                    std::string("Not found!")));
}
