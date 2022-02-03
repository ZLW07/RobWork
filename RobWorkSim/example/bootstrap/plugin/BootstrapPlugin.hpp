#ifndef BootstrapPlugin_HPP
#define BootstrapPlugin_HPP

#include <rws/RobWorkStudioPlugin.hpp>
#include "ui_BootstrapPlugin.h"

#include <QObject>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }
namespace rwsim { namespace simulator { class ThreadSimulator; } }

class Brain;
class QTimer;

/**
 * @brief A plugin
 */
class BootstrapPlugin: public rws::RobWorkStudioPlugin, private Ui::BootstrapPlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "BootstrapPlugin.json")
public:

    /**
     * @brief constructor
     */
	BootstrapPlugin();

    //! destructor
    virtual ~BootstrapPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);

    void makeSimulator();
    void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
    void startSimulation();

private slots:
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    rw::models::WorkCell* _wc;
    rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::core::Ptr<rwsim::simulator::ThreadSimulator> _tsim;
    rw::core::Ptr<rwsim::simulator::DynamicSimulator> _sim;

    rw::core::Ptr<Brain> _brain;

    QTimer *_timer;
};

#endif /*BootstrapPlugin_HPP*/
