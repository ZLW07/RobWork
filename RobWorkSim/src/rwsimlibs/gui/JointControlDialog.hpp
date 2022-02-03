#ifndef JOINTCONTROLDIALOG_HPP_
#define JOINTCONTROLDIALOG_HPP_

#include <rw/core/Ptr.hpp>

#include <QDialog>

namespace rwlibs { namespace control {
    class JointController;
}}    // namespace rwlibs::control
namespace rwsim { namespace dynamics {
    class DynamicDevice;
}}    // namespace rwsim::dynamics

class JogGroup;

class QTabWidget;

class JointControlDialog : public QDialog
{
    Q_OBJECT

  public:
    JointControlDialog (rw::core::Ptr< rwlibs::control::JointController > jcontroller,
                        QWidget* parent = 0);
    JointControlDialog (rw::core::Ptr< rwsim::dynamics::DynamicDevice > device,
                        QWidget* parent = 0);

    virtual ~JointControlDialog () {}

  private:
    QTabWidget* tabWidget;

    rw::core::Ptr< rwlibs::control::JointController > _controller;
    rw::core::Ptr< rwsim::dynamics::DynamicDevice > _device;
};

class SyncTab : public QWidget
{
    Q_OBJECT

  public:
    SyncTab (rw::core::Ptr< rwlibs::control::JointController > jcontroller, QWidget* parent = 0) :
        QWidget (parent)
    {}
    virtual ~SyncTab (){};
};

class PosTab : public QWidget
{
    Q_OBJECT

  public:
    PosTab (rw::core::Ptr< rwlibs::control::JointController > jcontroller, QWidget* parent = 0);

    virtual ~PosTab (){};

  private:
  private slots:
    void targetChanged ();
    void setTarget ();

  private:
    JogGroup* _jogGroup;
    rw::core::Ptr< rwlibs::control::JointController > _jcont;
};

class VelTab : public QWidget
{
    Q_OBJECT

  public:
    VelTab (rw::core::Ptr< rwsim::dynamics::DynamicDevice > device, QWidget* parent = 0) :
        QWidget (parent)
    {}

    VelTab (rw::core::Ptr< rwlibs::control::JointController > jcontroller, QWidget* parent = 0);

    virtual ~VelTab (){};
};

class CurTab : public QWidget
{
    Q_OBJECT

  public:
    CurTab (rw::core::Ptr< rwlibs::control::JointController > jcontroller, QWidget* parent = 0);
    virtual ~CurTab (){};
};

#endif /*CUBECONTROLDIALOG_HPP_*/
