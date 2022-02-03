#ifndef BODYCONTROLLERWIDGET_HPP_
#define BODYCONTROLLERWIDGET_HPP_

#include <rw/core/Ptr.hpp>

#include <QDialog>

namespace rwsim { namespace control {
    class BodyController;
}}    // namespace rwsim::control
// namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator {
    class DynamicSimulator;
}}    // namespace rwsim::simulator

class JogGroup;

/**
 * @brief dialog used to control dynamic bodies in the scene
 */
class BodyControlDialog : public QDialog
{
    Q_OBJECT

  public:
    BodyControlDialog (    // rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> dwc,
        rw::core::Ptr< rwsim::control::BodyController > bodycontroller, QWidget* parent = 0);

    BodyControlDialog (    // rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> dwc,
        rw::core::Ptr< rwsim::simulator::DynamicSimulator > simulator, QWidget* parent = 0);

    virtual ~BodyControlDialog () {}

  private:
    rw::core::Ptr< rwsim::control::BodyController > _bodyctrl;
    // rw::core::Ptr<rwsim::simulator::DynamicSimulator> _sim;
    JogGroup* _jogGroup;
};

#endif /* CONTROLLERWIDGET_HPP_ */
