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

#include "RobWorkStudioApp.hpp"

#ifdef _WIN32
#include <windows.h>
#endif    //#ifdef _WIN32

#include <RobWorkConfig.hpp>
#include <RobWorkStudioConfig.hpp>
#include <rw/common/ProgramOptions.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/RobWork.hpp>

#include <QApplication>
#include <QCloseEvent>
#include <QMessageBox>
#include <QSplashScreen>
#ifdef RWS_USE_STATIC_LINK_PLUGINS
#ifdef RWS_HAVE_PLUGIN_JOG
#include <rwslibs/jog/Jog.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_LOG
#include <rwslibs/log/ShowLog.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_PLANNING
#include <rwslibs/planning/Planning.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_PLAYBACK
#include <rwslibs/playback/PlayBack.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_PROPERTYVIEW
#include <rwslibs/propertyview/PropertyView.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_SENSORS
#include <rwslibs/sensors/Sensors.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_TREEVIEW
#include <rwslibs/treeview/TreeView.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_WORKCELLEDITOR
#include <rwslibs/workcelleditorplugin/WorkcellEditorPlugin.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_LUAPL
#include <rwslibs/lua/Lua.hpp>
#endif
#ifdef RWS_HAVE_PLUGIN_PYTHONEDITOR
#include <rwslibs/pythoneditor/Editor.hpp>
#endif
#endif
#ifdef RWS_HAVE_GLUT
#if defined(RW_MACOS)
//#include <GLUT/glut.h>
// TODO(kalor) Figure Out how to get GLUT to work as glutBitmapString is undeclared i mac
#undef RW_HAVE_GLUT
#else
#include <GL/freeglut.h>
#endif
#endif

#include <boost/filesystem.hpp>
#include <boost/program_options/parsers.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rws;

namespace {
class MyQApplication : public QApplication
{
  public:
    MyQApplication (int& argc, char** argv) : QApplication (argc, argv) {}

    bool notify (QObject* rec, QEvent* ev)
    {
        try {
            return QApplication::notify (rec, ev);
        }
        catch (std::exception& e) {
            QMessageBox::warning (0, tr ("An error occurred"), e.what ());
        }
        catch (...) {
            QMessageBox::warning (
                0, tr ("An unexpected error occurred"), tr ("This is likely a bug."));
        }
        return false;
    }
};
}    // namespace

RobWorkStudioApp::RobWorkStudioApp (const std::string& args) :
    _rwstudio (NULL), _args (args), _thread (NULL), _isRunning (false)
{
}

RobWorkStudioApp::~RobWorkStudioApp ()
{
    if (_isRunning) {
        QCloseEvent e = QCloseEvent ();
        _rwstudio->event (&e);
    }
}

void RobWorkStudioApp::start ()
{
    _thread = new boost::thread (boost::bind (&RobWorkStudioApp::run, this));
    while (!this->isRunning ()) {
        rw::common::TimerUtil::sleepMs (1);
    }
}

void RobWorkStudioApp::close ()
{
    if (isRunning ()) {
        _rwstudio->postExit ();
        while (isRunning ()) {
            rw::common::TimerUtil::sleepMs (1);
        }

        // Make Sure All Widgets are closed to avoid segfault
        QWidgetList all_w = QApplication::allWidgets ();
        long ctime        = rw::common::TimerUtil::currentTimeMs ();
        while (all_w.count () > 0 && rw::common::TimerUtil::currentTimeMs () - ctime < 300) {
            rw::common::TimerUtil::sleepMs (1);
            all_w = QApplication::allWidgets ();
        }
        rw::common::TimerUtil::sleepMs (1000);    // Final timing to let the rest of QT close down
    }
}

void initReasource ()
{
    Q_INIT_RESOURCE (rwstudio_resources);
}

namespace {
std::vector< std::string > split (std::string str, std::string token)
{
    std::vector< std::string > result;
    size_t index = str.find (token);
    if (index != std::string::npos) {
        while (str.size ()) {
            size_t index = str.find (token);
            if (index != std::string::npos) {
                result.push_back (str.substr (0, index));
                str = str.substr (index + token.size ());
                if (str.size () == 0)

                    result.push_back (str);
            }
            else {
                result.push_back (str);
                str = "";
            }
        }
    }
    else {
        result.push_back (str);
    }
    return result;
}

void loadPluginFolder (RobWorkStudio* rws, const std::string& folder,
                       const std::vector< std::string >& excludeList)
{
    if (boost::filesystem::exists (folder)) {
        boost::filesystem::path p2 (folder);
        for (boost::filesystem::directory_iterator i (p2);
             i != boost::filesystem::directory_iterator ();
             i++) {
            std::string plPath = std::string (folder) + "/" + i->path ().filename ().string ();

            bool exclude = false;
            for (const std::string& expl : excludeList) {
                if (plPath == expl || i->path ().filename ().string () == expl) {
                    exclude = true;
                    break;
                }
            }

            if (!exclude) {
                rws->loadPlugin (plPath.c_str (), 0, 1);
            }
        }
    }
}
}    // namespace

int RobWorkStudioApp::run ()
{
    initReasource ();

    char* argv[30];
    std::vector< std::string > args = boost::program_options::split_unix (_args);

    if (args.size () == 0) {
        args.push_back ("RobWorkStudio");
    }
    for (size_t i = 0; i < args.size (); i++) {
        argv[i] = &(args[i][0]);
    }

    int argc = (int) args.size ();
    // now initialize robwork, such that plugins and stuff might work

    if (argc == 0) {
        RobWork::init ();
    }
    else {
        RobWork::init (argc, argv);
    }

    ProgramOptions poptions ("RobWorkStudio", RW_VERSION);

    poptions.addStringOption ("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
    poptions.addStringOption ("input-file", "", "Project/Workcell/Device input file");
    poptions.addStringOption (
        "rwsplugin", "", "load RobWorkStudio plugin, not to be confused with '--rwplugin'");
    poptions.addStringOption ("nosplash", "", "If defined the splash screen will not be shown");
    poptions.addStringOption ("exclude-plugins", "", "list of plugins not to load seperated by ,");
    poptions.setPositionalOption ("input-file", -1);

    poptions.initOptions ();

    poptions.parse (argc, argv);

    PropertyMap map = poptions.getPropertyMap ();

    bool showSplash     = false;    //! map.has("nosplash");
    std::string inifile = map.get< std::string > ("ini-file", "");

    std::string inputfile = map.get< std::string > ("input-file", "");

    std::string rwsplugin = map.get< std::string > ("rwsplugin", "");
    std::vector< std::string > excludePl =
        split (map.get< std::string > ("exclude-plugins", ""), ",");

    {
        MyQApplication app (argc, argv);
#ifdef RWS_HAVE_GLUT
        glutInit (&argc, argv);
#endif

        try {
            QSplashScreen* splash;
            if (showSplash) {
                QPixmap pixmap (":/images/splash.jpg");
                splash = new QSplashScreen (pixmap);
                splash->show ();
                // Loading some items
                splash->showMessage ("Adding static plugins");
            }

            app.processEvents ();

            // Establishing connections

            if (showSplash)
                splash->showMessage ("Loading static plugins");

            std::string pluginFolder = "./plugins/";
            {
                Timer t;

                rws::RobWorkStudio rwstudio (map);

#ifdef RWS_USE_STATIC_LINK_PLUGINS
#ifdef RWS_HAVE_PLUGIN_LOG

                rwstudio.addPlugin (new rws::ShowLog (), false, Qt::BottomDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_JOG

                rwstudio.addPlugin (new rws::Jog (), false, Qt::LeftDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_TREEVIEW

                rwstudio.addPlugin (new rws::TreeView (), false, Qt::LeftDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_PLAYBACK

                rwstudio.addPlugin (new rws::PlayBack (), false, Qt::BottomDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_PROPERTYVIEW

                rwstudio.addPlugin (new rws::PropertyView (), false, Qt::LeftDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_PLANNING

                rwstudio.addPlugin (new rws::Planning (), false, Qt::LeftDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_SENSORS

                rwstudio.addPlugin (new rws::Sensors (), false, Qt::RightDockWidgetArea);
#endif
#ifdef RWS_HAVE_PLUGIN_WORKCELLEDITOR

                rwstudio.addPlugin (
                    new rws::WorkcellEditorPlugin (), false, Qt::LeftDockWidgetArea);
#endif
#ifdef RW_HAVE_EIGEN

                rwstudio.addPlugin (new rws::Calibration (), false, Qt::RightDockWidgetArea);
#endif

#if RWS_HAVE_PLUGIN_LUAPL

                rwstudio.addPlugin (new rws::Lua (), false, Qt::LeftDockWidgetArea);
#endif
#if RWS_HAVE_PLUGIN_PYTHONEDITOR
                rwstudio.addPlugin (new rws::PyEditor (), false, Qt::LeftDockWidgetArea);
#endif
#endif

                // Load all plugins from the local rwsplugins folder
                loadPluginFolder (&rwstudio, RWS_COMPILE_PLUGIN_DIR, excludePl);

                if (showSplash) {
                    splash->showMessage ("Loading static plugins");
                }

                rwstudio.loadSettingsSetupPlugins (inifile);
                // Load all plugins from the rwsplugins folder
                if (boost::filesystem::exists ("/usr/lib/")) {
                    boost::filesystem::path p ("/usr/lib");
                    // Find the architecture dependendt folder containing the
                    // rwsplugins folder
                    std::string rwspluginFolder = "";
                    for (boost::filesystem::directory_iterator i (p);
                         i != boost::filesystem::directory_iterator ();
                         i++) {
                        if (boost::filesystem::is_directory (i->path ())) {
                            rwspluginFolder = "/usr/lib/";
                            rwspluginFolder += i->path ().filename ().string ();
                            rwspluginFolder += "/RobWork/rwsplugins";
                            if (boost::filesystem::exists (rwspluginFolder)) {
                                break;
                            }
                            else {
                                rwspluginFolder = "";
                            }
                        }
                    }
                    // Load all plugins from the rwsplugins folder
                    loadPluginFolder (&rwstudio, rwspluginFolder, excludePl);
                }
                if (inputfile.empty ()) {
                    std::string workcellFile = rwstudio.loadSettingsWorkcell (inifile);
                    if (showSplash) {
                        splash->showMessage ("Opening workcell...");
                    }
                    rwstudio.openFile (workcellFile);
                }

                if (!inputfile.empty ()) {
                    if (showSplash)
                        splash->showMessage ("Opening workcell...");
                    rwstudio.openFile (inputfile);
                }

                if (!rwsplugin.empty ()) {
                    rwstudio.loadPlugin (rwsplugin);
                }

                // load configuration into RobWorkStudio
                if (showSplash) {
                    splash->showMessage ("Loading settings");
                    splash->finish (&rwstudio);
                }

                _rwstudio = &rwstudio;
                rwstudio.show ();
                _isRunning = true;

                app.exec ();
                _isRunning = false;
                _rwstudio  = NULL;
            }
        }
        catch (const Exception& e) {
            std::cout << e.what () << std::endl;
            QMessageBox::critical (NULL, "RW Exception", e.what ());
            _isRunning = false;
            return -1;
        }
        catch (std::exception& e) {
            std::cout << e.what () << std::endl;
            QMessageBox::critical (NULL, "Exception", e.what ());
            _isRunning = false;
            return -1;
        }
        catch (int) {
        }
    }
    _isRunning = false;
    return 0;
}