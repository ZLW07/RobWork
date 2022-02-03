#include "PyPlugin.hpp"

#include <RobWorkStudioConfig.hpp>
#include <rw/kinematics.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/pythonpluginloader/PythonRunner.hpp>

#include <QFile>
#include <QGridLayout>
#include <QTextStream>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

using rws::RobWorkStudioPlugin;
using namespace rw::kinematics;


PyPlugin::PyPlugin (const QString& name, const QIcon& icon) :
    RobWorkStudioPlugin (std::string (name.toStdString () + std::to_string (_pyPlugins++)).c_str (),
                         icon),
    _isPythonInit (false)
{
    _base                = new QWidget (this);
    _pluginName          = name.toStdString () + std::to_string (_pyPlugins - 1);
    QGridLayout* pLayout = new QGridLayout (_base);
    _base->setLayout (pLayout);
    this->setWidget (_base);
}

PyPlugin::~PyPlugin ()
{}

bool PyPlugin::initialize (std::string pythonFilePath, std::string pluginName)
{
    bool exsist = boost::filesystem::exists (pythonFilePath);
    if (exsist) {
        getRobWorkStudio ()->stateChangedEvent ().add (
            boost::bind (&PyPlugin::stateChangedListener, this, boost::arg< 1 > ()), this);

        _base->setObjectName (pluginName.c_str ());
        // Python_RWS_plugin_init
        QString fileName (":/PyPlugin.py");
        QFile file (fileName);
        if (file.open (QIODevice::ReadOnly)) {
            QTextStream in (&file);
            QString text = in.readAll ();
            _python.runCode (text.toStdString ().c_str ());
        }
        else {
            RW_THROW ("Could not open PyPlugin.py");
        }

        _isPythonInit = true;

        _python.runCode ("rws_cpp_link.new_widget('" + pluginName + "')\n");
        // Get Python
        std::ifstream scriptFile (pythonFilePath.c_str ());
        std::string code ((std::istreambuf_iterator< char > (scriptFile)),
                          std::istreambuf_iterator< char > ());

        _python.runCode (code.c_str ());
    }
    return exsist;
}

void PyPlugin::open (rw::models::WorkCell* workcell)
{
    if (_isPythonInit) {
        _python.runCode ("rws_cpp_link.openWorkCell()\n");
    }
}

void PyPlugin::close ()
{
    if (_isPythonInit) {
        _python.runCode ("rws_cpp_link.closeWorkCell()\n");
    }
}

void PyPlugin::stateChangedListener (const State& state)
{
    if (_isPythonInit) {
        _python.runCode ("rws_cpp_link.stateChanged()\n");
    }
}

size_t PyPlugin::_pyPlugins = 0;
