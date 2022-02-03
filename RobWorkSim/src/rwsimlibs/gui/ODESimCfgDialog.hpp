/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef ODESIMCFGDIALOG_HPP_
#define ODESIMCFGDIALOG_HPP_

#include <rw/core/Ptr.hpp>

#include <QDialog>
#include <QObject>

namespace rwsim { namespace simulator {
    class DynamicSimulator;
}}    // namespace rwsim::simulator

namespace Ui {
class ODESimCfgForm;
}

class ODESimCfgDialog : public QDialog
{
    Q_OBJECT

  public:
    ODESimCfgDialog (rw::core::Ptr< rwsim::simulator::DynamicSimulator > sim, QWidget* parent = 0);

    void initializeStart ();

  signals:
    void updateView ();

  private slots:
    void btnPressed ();
    void changedEvent ();

  private:
    void updateStatus ();
    void applyChanges ();
    void updateValues ();

  private:
    Ui::ODESimCfgForm* _ui;

    rw::core::Ptr< rwsim::simulator::DynamicSimulator > _sim;
};

#endif /* ODESimCfgDialog_HPP_ */
