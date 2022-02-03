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

#include "Task.xml.hpp"

#include <RobWorkConfig.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/TaskSaver.hpp>

#include <fstream>

using rw::core::ownedPtr;
using rw::math::Q;
using namespace rwlibs::task;

namespace {
    QTask::Ptr getTask()
    {
        const QTask::Ptr task = ownedPtr(new QTask());
        Q q1(1); q1(0) = 1;
        Q q2(1); q2(0) = 2;
        Q q3(1); q3(0) = 3;
        task->addTargetByValue(q1);
        task->addTargetByValue(q2);
        task->addTargetByValue(q3);

        std::vector<QTarget::Ptr>& targets = task->getTargets();
        task->addMotion(ownedPtr(new QP2PMotion(targets[0], targets[1])));
        task->addMotion(ownedPtr(new QLinearMotion(targets[1], targets[2])));
        task->addAction(ownedPtr(new Action(ActionType::On)));

        return task;
    }
}

TEST(TaskSaver, DOMXMLtoFile)
{
    const QTask::Ptr task = getTask();
    const TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml", "DOM");
    saver->save(task, "TaskSaverTestDOM.xml");
    std::ifstream file("TaskSaverTestDOM.xml", std::ios_base::in);
    std::stringstream stream;
    std::string line;
    while (std::getline(file, line))
        stream << line << std::endl;
    file.close();
    EXPECT_EQ(getTaskXML(), stream.str());
}

TEST(TaskSaver, DOMXMLtoStream)
{
    const QTask::Ptr task = getTask();
    const TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml", "DOM");
    std::stringstream sstr;
    saver->save(task, sstr);
    EXPECT_EQ(getTaskXML(), sstr.str());
}

#ifdef RW_HAVE_XERCES
TEST(TaskSaver, XercesXMLtoFile)
{
    const QTask::Ptr task = getTask();
    const TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml", "XERCES");
    saver->save(task, "TaskSaverTestXerces.xml");
    std::ifstream file("TaskSaverTestXerces.xml", std::ios_base::in);
    std::stringstream stream;
    std::string line;
    while (std::getline(file, line))
        stream << line << std::endl;
    file.close();
    EXPECT_EQ(getTaskXMLXerces(), stream.str());
}

TEST(TaskSaver, XercesXMLtoStream)
{
    const QTask::Ptr task = getTask();
    const TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml", "XERCES");
    std::stringstream sstr;
    saver->save(task, sstr);
    EXPECT_EQ(getTaskXMLXerces(), sstr.str());
}
#endif
