#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_SamplePlugin.h"

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);
};

#endif /*SAMPLEPLUGIN_HPP*/
