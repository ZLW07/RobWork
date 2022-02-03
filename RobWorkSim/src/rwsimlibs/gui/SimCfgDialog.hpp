/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef SIMULATORCFGDIALOG_HPP_
#define SIMULATORCFGDIALOG_HPP_

#include <rw/core/Ptr.hpp>

#include <QDialog>
#include <QObject>

namespace rwsim { namespace simulator {
    class DynamicSimulator;
}}    // namespace rwsim::simulator

namespace Ui {
class SimCfgDialog;
}

class SimCfgDialog : public QDialog
{
    Q_OBJECT

  public:
    SimCfgDialog (rw::core::Ptr< rwsim::simulator::DynamicSimulator > sim, QWidget* parent = 0);

  private slots:
    void btnPressed ();
    void changedEvent ();

  private:
    Ui::SimCfgDialog* _ui;
};

#endif /* SimulatorCfgDialog_HPP_ */
