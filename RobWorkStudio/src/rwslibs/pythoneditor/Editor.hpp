#ifndef RWSLIBS_PTHONEDITOR_PYEDITOR_HPP
#define RWSLIBS_PTHONEDITOR_PYEDITOR_HPP

#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/pythonpluginloader/PythonRunner.hpp>

class QCodeEditor;
namespace rws {

/**
 * @brief This plugin can be used to edit python code and run the code directly from RobWorkStudio
 */
class PyEditor : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
    Q_PLUGIN_METADATA (IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
  public:
    /**
     * @brief constructor
     */
    PyEditor ();

    /**
     * @brief destructor
     */
    virtual ~PyEditor ();

    /**
     * @copydoc RobWorkStudioPlugin::open
     */
    virtual void open (rw::models::WorkCell* workcell);

      /**
     * @copydoc RobWorkStudioPlugin::close
     */
    virtual void close ();

      /**
     * @copydoc RobWorkStudioPlugin::initialize
     */
    virtual void initialize ();

  private Q_SLOTS:
    void stateChangedListener (const rw::kinematics::State& state);

    void runCode();

  private:
    QCodeEditor* _editor;
    rws::python::PythonRunner _pyrun;
};
}    // namespace rws
#endif
