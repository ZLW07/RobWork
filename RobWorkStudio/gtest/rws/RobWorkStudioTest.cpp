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

#include <rw/core/PropertyMap.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <QApplication>
#include <QString>
#include <gtest/gtest.h>

using namespace rw::core;
using namespace rw::common;
using namespace rws;

TEST (RobWorkStudio, LaunchTest)
{
    int argc      = 1;
    char name[]   = "RobWorkStudio";
    char* argv[1] = {name};
    PropertyMap map;
    QApplication app (argc, argv);
    RobWorkStudio rwstudio (map);
    rwstudio.show ();
    app.processEvents ();
    TimerUtil::sleepMs (1000);
    rwstudio.close ();
    TimerUtil::sleepMs (2000);
}

#ifndef WIN32
TEST (RobWorkStudio, PluginLoadTest)
{
    rws::RobWorkStudioApp rwsApp ("");
    rwsApp.start ();
    rws::RobWorkStudio* rwstudio = rwsApp.getRobWorkStudio ();
    TimerUtil::sleepMs (1000);
    std::vector< rws::RobWorkStudioPlugin* > pl = rwstudio->getPlugins ();

    std::vector< QString > plugins = {"ATaskVisPlugin",
                                      "PlayBack",
                                      "Jog",
                                      "Workcell Editor",
                                      "Log",
                                      "LuaConsole",
                                      "TreeView",
                                      "Planning",
                                      "PropertyView",
                                      "Sensors",
                                      "GTaskVisPlugin"};
    for (QString& pn : plugins) {
        bool exist = false;
        for (rws::RobWorkStudioPlugin* p : pl) {
            if (p->name () == pn) {
                exist = true;
                break;
            }
        }
        if (!exist) {    // Print som debug output
            std::cout << "Could not find '" << pn.toStdString () << "' in list: " << std::endl;
            for (rws::RobWorkStudioPlugin* p : pl) {
                std::cout << p->name ().toStdString () << ", ";
            }
            std::cout << std::endl;
        }

        EXPECT_TRUE (exist);
    }

    rwsApp.close ();
    TimerUtil::sleepMs (1000);
}
#endif