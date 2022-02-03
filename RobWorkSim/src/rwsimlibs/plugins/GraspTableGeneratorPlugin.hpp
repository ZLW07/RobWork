/*
 * GraspTableGeneratorPlugin.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GraspTableGeneratorPlugin_HPP_
#define GraspTableGeneratorPlugin_HPP_

#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/State.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <rwsimlibs/gui/ThreadSafeStack.hpp>

#include <Eigen/Core>

namespace rw { namespace graspplanning {
    class GraspTable;
}}    // namespace rw::graspplanning
namespace rw { namespace kinematics {
    class MovableFrame;
}}    // namespace rw::kinematics
namespace rw { namespace proximity {
    class CollisionDetector;
}}    // namespace rw::proximity
namespace rwsim { namespace control {
    class PDController;
}}    // namespace rwsim::control
namespace rwsim { namespace dynamics {
    class DynamicWorkCell;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class RigidBody;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class RigidDevice;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace sensor {
    class BodyContactSensor;
}}    // namespace rwsim::sensor
namespace rwsim { namespace simulator {
    class DynamicSimulator;
}}    // namespace rwsim::simulator
namespace rwsim { namespace simulator {
    class ThreadSimulator;
}}    // namespace rwsim::simulator
namespace rwsim { namespace util {
    class GraspPolicy;
}}    // namespace rwsim::util
namespace rwsim { namespace util {
    class GraspStrategy;
}}    // namespace rwsim::util
namespace rwsim { namespace util {
    class RestingPoseGenerator;
}}    // namespace rwsim::util

namespace Ui {
class GraspTableGeneratorPlugin;
}

struct RestingConfig
{
    RestingConfig (const rw::kinematics::State& state, const std::string& str) :
        _state (state), _desc (str)
    {}
    RestingConfig (){};
    rw::kinematics::State _state;
    std::string _desc;
};
/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class GraspTableGeneratorPlugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
    Q_PLUGIN_METADATA (IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE
                           "GraspTableGeneratorPlugin.json")
  public:
    typedef std::vector< Eigen::MatrixXf > TactileSensorData;

    /**
     * @brief constructor
     */
    GraspTableGeneratorPlugin ();

    /**
     * @brief destructor
     */
    virtual ~GraspTableGeneratorPlugin ();

    /**
     * @brief starts the generation of the grasp table
     */
    void startTableGeneration ();

    /**
     * @brief callback used for interfacing to ThreadSimulator
     */
    void stepCallBack (int i, const rw::kinematics::State& state);

    /**
     * @brief for state changes of RWS
     */
    void stateChangedListener (const rw::kinematics::State& state);

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener (const std::string& event);

    ////// inherited from RobWorkStudioPlugin

    //! @copydoc rws::RobWorkStudioPlugin::open
    void open (rw::models::WorkCell* workcell);

    //! @copydoc rws::RobWorkStudioPlugin::close
    void close ();

    //! @copydoc rws::RobWorkStudioPlugin::initialize
    void initialize ();

  private slots:
    void btnPressed ();
    void changedEvent ();

  private:
    /*
    void initializeStart();
    void updateStatus();

    bool isSimulationFinished( SimulatorPtr sim, const rw::kinematics::State& state );
    bool saveRestingState( int simidx, SimulatorPtr sim , const rw::kinematics::State& state );
     */

    void cleanup ();
    void loadConfiguration (const std::string& filename);
    void saveConfiguration (const std::string& filename);
    void applyConfiguration ();
    void readConfiguration ();

  private:
    rw::core::PropertyMap _settings;

    struct CallBackFunctor
    {
        CallBackFunctor (int i, GraspTableGeneratorPlugin* parent) : _i (i), _parent (parent) {}

        void stepCallBack (const rw::kinematics::State& state)
        {
            _parent->stepCallBack (_i, state);
        }

        int _i;
        GraspTableGeneratorPlugin* _parent;
    };

    std::vector< rwsim::util::RestingPoseGenerator* > _generators;

    Ui::GraspTableGeneratorPlugin* _ui;

    rw::kinematics::State _defstate;
    rw::kinematics::State _state;
    QTimer* _timer;
    std::vector< rw::core::Ptr< rwsim::simulator::ThreadSimulator > > _simulators;
    std::vector< rw::kinematics::State > _initStates;
    std::vector< double > _simStartTimes;
    int _nrOfTests;
    double _totalSimTime;
    std::vector< rwsim::dynamics::RigidBody* > _bodies;

    long _startTime;

    std::vector< rw::kinematics::State > _startPoses;
    std::vector< rw::kinematics::State > _resultPoses;

    rw::kinematics::FrameMap< rwsim::dynamics::RigidBody* > _frameToBody;
    rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > _dwc;

    rw::proximity::CollisionDetector* _colDect;
    double _lastTime, _lastBelowThresUpdate;
    rwsim::util::MovingAverage _avgSimTime;
    rwsim::util::MovingAverage _avgTime;

    std::vector< rw::core::Ptr< rwsim::control::PDController > > _controllers;
    std::vector< rw::math::Q > _preshapes;
    std::vector< rw::math::Q > _targetQ;
    rwsim::dynamics::RigidBody* _body;
    rwsim::dynamics::RigidDevice* _hand;
    rw::kinematics::MovableFrame *_handBase, *_object;

    rw::core::Ptr< rwsim::sensor::BodyContactSensor > _bodySensor;

    bool _exitHard;

    bool _graspNotStable;

    std::vector< bool > _fingersInContact;

    std::vector< std::vector< rw::math::Q > > _handconfigs;
    std::vector< std::vector< TactileSensorData > > _tactiledatas;

    std::vector< rw::core::Ptr< CallBackFunctor > > _functors;
    std::vector< double > _nextTimeUpdate;
    int _nrOfTestsOld;

    ThreadSafeStack< RestingConfig > _restingConfigs;

    // QSampler *_handQSampler;

    std::vector< int > _currentPreshapeIDX;
    rw::math::Q _target, _preshape;
    rw::math::Transform3D<> _objTransform;

    int _nrOfGraspsInGroup, _lastTableBackupCnt;
    int _tactileDataOnAllCnt;

    rw::core::Ptr< rwsim::util::GraspStrategy > _gstrategy;
    rw::core::Ptr< rwsim::util::GraspPolicy > _gpolicy;
    rw::core::Ptr< rwsim::simulator::DynamicSimulator > _simulator;

    rw::graspplanning::GraspTable* _gtable;
    std::string _configFile;    // loadet on initialization
    rw::core::PropertyMap _config;
};

#endif /* RESTINGPOSEDIALOG_HPP_ */
