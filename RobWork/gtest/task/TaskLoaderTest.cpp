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

#include "../TestEnvironment.hpp"

#include "Task.xml.hpp"

#include <RobWorkConfig.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/TaskLoader.hpp>

#include <fstream>

using rw::core::ownedPtr;
using rw::math::Q;
using namespace rwlibs::task;

namespace {
    void checkTask(const QTask::Ptr task)
    {
        const std::vector<QTarget::Ptr>& targets = task->getTargets();
        const std::vector<QMotion::Ptr>& motions = task->getMotions();
        const std::vector<Action::Ptr>& actions = task->getActions();
        ASSERT_EQ(std::size_t(3), targets.size());
        ASSERT_EQ(std::size_t(2), motions.size());
        ASSERT_EQ(std::size_t(1), actions.size());
        const QTarget::Ptr target1 = targets[0];
        const QTarget::Ptr target2 = targets[1];
        const QTarget::Ptr target3 = targets[2];
        ASSERT_FALSE(target1.isNull());
        ASSERT_FALSE(target2.isNull());
        ASSERT_FALSE(target3.isNull());
        Q q1 = target1->get();
        Q q2 = target2->get();
        Q q3 = target3->get();
        ASSERT_EQ(std::size_t(1), q1.size());
        ASSERT_EQ(std::size_t(1), q2.size());
        ASSERT_EQ(std::size_t(1), q3.size());
        EXPECT_EQ(1., q1[0]);
        EXPECT_EQ(2., q2[0]);
        EXPECT_EQ(3., q3[0]);
        const QP2PMotion::Ptr motion1 = motions[0].cast<QP2PMotion>();
        const QLinearMotion::Ptr motion2 = motions[1].cast<QLinearMotion>();
        ASSERT_FALSE(motion1.isNull());
        ASSERT_FALSE(motion2.isNull());
        EXPECT_EQ(target1, motion1->startTarget());
        EXPECT_EQ(target2, motion1->endTarget());
        EXPECT_EQ(target2, motion2->startTarget());
        EXPECT_EQ(target3, motion2->endTarget());
        const Action::Ptr action = actions[0];
        ASSERT_FALSE(action.isNull());
        EXPECT_EQ(ActionType::On, action->actionType());
    }
}

TEST(TaskLoader, DOMXMLfromFile)
{
    std::ofstream file("TaskLoader.xml", std::ios_base::out);
    file << getTaskXML();
    file.close();
    const TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml", "DOM");
    loader->load("TaskLoader.xml");
    const QTask::Ptr task = loader->getQTask();
    ASSERT_FALSE(task.isNull());
    checkTask(task);
}

TEST(TaskLoader, DOMXMLfromStream)
{
    const TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml", "DOM");
    std::stringstream stream;
    stream << getTaskXML();
    loader->load(stream);
    const QTask::Ptr task = loader->getQTask();
    ASSERT_FALSE(task.isNull());
    checkTask(task);
}

#ifdef RW_HAVE_XERCES
TEST(TaskLoader, XercesXMLfromFile)
{
    std::ofstream file("TaskLoaderXerces.xml", std::ios_base::out);
    file << getTaskXMLXerces();
    file.close();
    const TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml", "XERCES");
    loader->load("TaskLoaderXerces.xml", TestEnvironment::xmlSchemasDir() + "/rwxml_task.xsd");
    const QTask::Ptr task = loader->getQTask();
    ASSERT_FALSE(task.isNull());
    checkTask(task);
}

TEST(TaskLoader, XercesXMLfromStream)
{
    const TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml", "XERCES");
    std::stringstream stream;
    stream << getTaskXML();
    loader->load(stream, TestEnvironment::xmlSchemasDir() + "/rwxml_task.xsd");
    const QTask::Ptr task = loader->getQTask();
    ASSERT_FALSE(task.isNull());
    checkTask(task);
}
#endif
