#ifndef SimTaskVisPlugin_HPP
#define SimTaskVisPlugin_HPP

#include <rwlibs/task/Task.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include "ui_SimTaskVisPlugin.h"

#include <QObject>

namespace rw { namespace graphics { class Render; } }
namespace rw { namespace kinematics { class MovableFrame; } }
namespace rw { namespace models { class Device; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace dynamics { class RigidBody; } }

/**
 * @brief A plugin that continuesly grasps an object from a target pose whereafter it is
 * lifted to a home pose.
 *
 * The home and target poses are controlled through a task description file. Which is
 * allso used to write back all the results of the simulation.
 *
 * The configuration of the simulation is setup through properties. These can be set from the
 * command prompt, loaded by file, or edited in the gui. These properties include:
 *
 * - Simulator
 * - TimeStepSize
 * - HandOpenConfig
 * - HandCloseConfig
 * - MinRestingTime
 *
 *
 */
class SimTaskVisPlugin: public rws::RobWorkStudioPlugin, private Ui::SimTaskVisPlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "SimTaskVisPlugin.json")
public:
    SimTaskVisPlugin();
	virtual ~SimTaskVisPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);


    void loadTasks(bool automatic);
    void saveTasks(bool automatic);
    void loadConfig(bool automatic);
    void saveConfig();
    void updateConfig();

    rw::core::PropertyMap& settings();

    //std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state);

    // for task iteration
    bool hasNextTarget();
    rwlibs::task::CartesianTarget::Ptr getNextTarget();
    void setTask(int);
    rwlibs::task::CartesianTask::Ptr getTask();
    rwlibs::task::CartesianTarget::Ptr getTarget();
private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    typedef enum Status {UnInitialized = 0, Success, CollisionInitially, ObjectMissed, ObjectDropped, ObjectSlipped, TimeOut, SimulationFailure} TestStatus;

    rw::models::WorkCell* _wc;
    rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;

    rw::models::Device* _hand;
    rw::kinematics::MovableFrame *_mbase;
    rw::kinematics::Frame *_tcp;
    std::vector<rw::core::Ptr<rwsim::dynamics::RigidBody> > _objects;

    rwlibs::task::CartesianTask::Ptr _roottask,_currenttask;
    std::vector<rwlibs::task::CartesianTask::Ptr> _taskQueue;
    std::vector<rwlibs::task::CartesianTarget::Ptr> *_targets;
    int _currentTaskIndex;
    int _currentTargetIndex, _nextTargetIndex;
    std::size_t _totalNrOfExperiments;

    std::vector<std::pair<rwlibs::task::CartesianTask::Ptr, rwlibs::task::CartesianTarget::Ptr> > _ymtargets;

    rw::core::PropertyMap _config;

    rw::core::Ptr<rw::graphics::Render> _render;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
