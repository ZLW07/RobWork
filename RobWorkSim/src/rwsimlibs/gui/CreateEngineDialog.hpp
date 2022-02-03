/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef CREATEENGINEDIALOG_HPP_
#define CREATEENGINEDIALOG_HPP_

#include <rw/core/Ptr.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

#include <QDialog>
#include <QObject>

namespace rwsim { namespace dynamics {
    class DynamicWorkCell;
}}    // namespace rwsim::dynamics

namespace Ui {
class CreateEngineDialog;
}

class CreateEngineDialog : public QDialog
{
    Q_OBJECT

  public:
    CreateEngineDialog (rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc, QWidget* parent = 0);

    rwsim::simulator::DynamicSimulator::Ptr getSimulator () { return _sim; };

  private slots:
    void btnPressed ();
    void changedEvent ();

  private:
    Ui::CreateEngineDialog* _ui;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > _dwc;
};

#endif /* CreateEngineDialog_HPP_ */
