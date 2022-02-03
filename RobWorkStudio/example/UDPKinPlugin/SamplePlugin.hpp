#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/math/Transform3D.hpp>

#include <boost/asio.hpp>

#include "ui_SamplePlugin.h"

namespace rw { namespace graphics { class DrawableNode; } }
namespace rw { namespace models { class Device; } }
namespace rwlibs { namespace opengl { class RenderFrame; } }

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
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
    void readUpdateFromUDP();
private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    QTimer *_timer;

    boost::asio::io_service *io_service;
    boost::asio::ip::udp::endpoint* endpoint;
    boost::asio::ip::udp::socket* socket;
    boost::asio::ip::udp::resolver *resolver;

    rw::core::Ptr<rwlibs::opengl::RenderFrame> _renderFrame;
    std::vector<rw::core::Ptr<rw::graphics::DrawableNode> > _drawables;

    std::vector<rw::math::Transform3D<> > _transforms;
    std::vector<std::string> _toFrameName;

    rw::core::Ptr<rw::models::Device> _rightArm, _leftArm;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
