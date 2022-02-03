#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>

class QPushButton;

class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "SamplePlugin.json")
public:
    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    QPushButton* _btn0,*_btn1;
};

#endif /*SAMPLEPLUGIN_HPP*/
