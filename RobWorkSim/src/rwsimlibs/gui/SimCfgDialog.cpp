#include "SimCfgDialog.hpp"

//

#include "ODESimCfgDialog.hpp"
#include "ui_SimCfgDialog.h"

#include <RobWorkSimConfig.hpp>
#include <rw/core/Ptr.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

using namespace rwsim::simulator;

SimCfgDialog::SimCfgDialog (rw::core::Ptr< DynamicSimulator > sim, QWidget* parent) :
    QDialog (parent)
{
    _ui = new Ui::SimCfgDialog ();
    _ui->setupUi (this);

    connect (_ui->_applyBtn, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_ui->_cancelBtn, SIGNAL (pressed ()), this, SLOT (btnPressed ()));

    std::vector< std::string > engineIDs = PhysicsEngine::Factory::getEngineIDs ();

    // std::string id = _sim->getID();
    //_tabPane->addItem(id);
#ifdef RWSIM_HAVE_ODE
    ODESimCfgDialog* dialog = new ODESimCfgDialog (sim, this);
    _ui->_tabPane->addTab (dialog, "ODE");
#endif
    // for(const std::string& engineID: engineIDs){
    //_tabPane->addTab(this, engineID.c_str() );
    //_spaceMethodBox->addItem(engineID.c_str());
    //}

    // if(engineIDs.size()==0)
    //	_createBtn->setDisabled(true);
    //_cancelBtn->setDisabled(true);
}

void SimCfgDialog::btnPressed ()
{
    QObject* obj = sender ();
    if (obj == _ui->_applyBtn) {
    }
    else if (obj == _ui->_cancelBtn) {
    }
}

void SimCfgDialog::changedEvent ()
{
    // QObject *obj = sender();
    /*
    if( obj == _timer ){

    }
    */
}
