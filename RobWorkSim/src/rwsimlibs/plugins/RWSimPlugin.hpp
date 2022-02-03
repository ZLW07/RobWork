/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef RWSIMPLUGIN_HPP_
#define RWSIMPLUGIN_HPP_

#include "ui_RWSimPlugin.h"

#include <RobWorkSimConfig.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/trajectory/Path.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#ifdef RWSIM_PLUGIN_HAVE_LUA
#include <rwlibs/swig/lua/LuaState.hpp>
#include <rwsimlibs/swig/ScriptTypes.hpp>
#include <rwsimlibs/swig/lua/Lua.hpp>
#endif

#include <QObject>

namespace rwlibs { namespace opengl {
    class Drawable;
}}    // namespace rwlibs::opengl
namespace rwsim { namespace drawable {
    class SimulatorDebugRender;
}}    // namespace rwsim::drawable
namespace rwsim { namespace dynamics {
    class DynamicWorkCell;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace simulator {
    class ThreadSimulator;
}}    // namespace rwsim::simulator

class TactileSensorDialog;

class QTimer;

struct UserContext
{
    std::string _previousOpenDirectory;
};

/**
 * @brief A plugin for loading dynamic workcells and for doing simple
 * dynamics simulation using different physics engines.
 *
 *
 */
class RWSimPlugin : public rws::RobWorkStudioPlugin, private Ui::RWSimPlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
    Q_PLUGIN_METADATA (IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "RWSimPlugin.json")
  public:
    /**
     * @brief constructor
     */
    RWSimPlugin ();

    /**
     * @brief destructor
     */
    virtual ~RWSimPlugin ();

    /**
     * @copydoc rws::RobWorkStudioPlugin::open
     */
    void open (rw::models::WorkCell* workcell);

    /**
     * @copydoc rws::RobWorkStudioPlugin::close
     */
    void close ();

    /**
     * @copydoc rws::RobWorkStudioPlugin::initialize
     */
    void initialize ();

    /**
     *
     */
    void stateChangedListener (const rw::kinematics::State& state);

    /**
     * @brief opens a dynamic workcell with filename \b file.
     *
     * @note if successfully openned a generic event with string id
     * 'DynamicWorkCellLoaded' will be emitted and the DynamicWorkCell will
     * be saved in the RobWorkStudio propertymap with the string id 'DynamicWorkcell'.
     */
    void openDwc (const std::string& file);

    void stepCallBack (const rw::kinematics::State& state);

    void setupMenu (QMenu* menu);

  signals:
    void updateView ();
    // void updateDialog();
    void updateDialog (const rw::kinematics::State& state);

  private slots:
    void btnPressed ();
    void changedEvent ();
    void setRobWorkStudioState (const rw::kinematics::State& state);

  protected:
    void updateStatus ();

    rw::core::PropertyMap& settings ();

  private:
#ifdef RWSIM_PLUGIN_HAVE_LUA
    rwlibs::swig::LuaState::Ptr _luastate;
#endif

    Ui::RWSimPlugin _ui;

    QTimer* _timer;
    UserContext _context;
    rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > _dwc;
    rw::core::Ptr< rwsim::simulator::ThreadSimulator > _sim;

    rw::kinematics::State _state;

    long _startTime;

    rw::trajectory::TimedStatePath _path;
    rwlibs::opengl::Drawable* _debugDrawable;
    rw::core::Ptr< rwsim::drawable::SimulatorDebugRender > _debugRender;

    bool _openCalled;

    TactileSensorDialog* _tactileSensorDialog;

    QTimer* _timerShot;

    QAction *_openAction, *_planarPoseDistAction, *_poseDistAction, *_graspSelectionAction,
        *_graspRestPoseAction, *_restPoseAction, *_poseAnalyserAction;
};

#endif /* RESTINGPOSEDIALOG_HPP_ */
