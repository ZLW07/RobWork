
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

#ifndef RWS_PYTHONPLUGIN_PYPLUGIN_HPP_
#define RWS_PYTHONPLUGIN_PYPLUGIN_HPP_

#include <rw/kinematics/State.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/pythonpluginloader/PythonRunner.hpp>

#include <string>

class PyPlugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
  public:
    //! @copydoc rws::RobWorkStudioPlugin::RobWorkStudioPlugin
    PyPlugin (const QString& name, const QIcon& icon);

    /**
     * @brief Initialize the Python plugin
     * @param pythonFilePath [in] Full path to the python file
     * @param pluginName [in] the name of the plugin to be used when locating
     * the widget in python
     *
     * @return true if succes otherwise false
     */
    bool initialize (std::string pythonFilePath, std::string pluginName);

    void open (rw::models::WorkCell* workcell);

    void close ();

  protected:
    /**
     * @brief simple destructer
     */
    ~PyPlugin ();
  private Q_SLOTS:

    void stateChangedListener (const rw::kinematics::State& state);

  private:
    static size_t _pyPlugins;
    std::string _pythonFilePath;
    std::string _pluginName;
    QWidget* _base;
    bool _isPythonInit;
    rws::python::PythonRunner _python;
};

#endif /*SAMPLEPLUGIN_HPP*/
