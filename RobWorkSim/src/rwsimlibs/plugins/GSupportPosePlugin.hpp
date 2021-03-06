/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef SUPPORTPOSEANALYSERDIALOG_HPP_
#define SUPPORTPOSEANALYSERDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "GLViewRW.hpp"
#include "RestingPoseDialog.hpp"
#include "ui_SupportPoseAnalyserDialog.h"

#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Pose6D.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/drawable/RenderFrame.hpp>
#include <rwsim/drawable/RenderCircles.hpp>
#include <rwsim/drawable/RenderPlanes.hpp>
#include <rwsim/drawable/RenderPoints.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/util/CircleModel.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <rwsim/util/SupportPose.hpp>

#include <QObject>
#include <QTimer>
#include <QtGui>

/**
 * @brief gaphical user interface for calculating support pose and related
 * statistics of multiple objects on some support structure.
 */
class SupportPoseAnalyserDialog : public QDialog, private Ui::SupportPoseAnalyserDialog
{
    Q_OBJECT

  public:
    /**
     * @brief
     */
    SupportPoseAnalyserDialog (const rw::kinematics::State& state,
                               rwsim::dynamics::DynamicWorkCell* dwc,
                               rw::proximity::CollisionDetector* detector,
                               rws::RobWorkStudio* _rwstudio, QWidget* parent = 0);

    virtual ~SupportPoseAnalyserDialog (){};

    void initializeStart ();

  signals:
    void stateChanged (const rw::kinematics::State& state);

  private slots:
    void btnPressed ();
    void changedEvent ();
    void addRestingPose (const rw::kinematics::State& startstate,
                         const rw::kinematics::State& reststate);

  private:
    void updateStatus ();
    void addStatePath (rw::trajectory::TimedStatePathPtr path);
    void addStateStartPath (rw::trajectory::TimedStatePathPtr path);
    void process ();
    void updateRenderView ();
    void updateResultView ();

    void showPlanarDistribution ();
    void saveDistribution ();

    void updateHoughThres ()
    {
        // we set the thres hold such that [30,50] maps to [100,1000]
        int samples  = _xaxis[0].size ();
        double a     = ((50.0 - 30.0) / (1000.0 - 100.0));
        double b     = 30 - 100.0 * a;
        double thres = a * samples + b;
        thres        = rw::math::Math::clamp (thres, 20.0, 250.0);
        _thresholdSpin->setValue ((int) thres);
    }

    rwsim::dynamics::RigidBody* getSelectedBody ()
    {
        int i = _selectObjBox->currentIndex ();
        return _bodies[i];
    }

  private:
    Ui::SupportPoseAnalyserDialog _ui;
    std::string _previousOpenDirectory;

    rw::kinematics::State _defaultState;
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell* _dwc;
    rw::proximity::CollisionDetector* _detector;

    rw::trajectory::TimedStatePathPtr _path, _startPath;
    GLViewRW* _view;

    rw::core::Ptr< rw::graphics::RenderFrame > _frameRender;
    rwlibs::drawable::Drawable *_fDraw, *_fDraw1, *_fDraw2, *_fDraw3, *_fDraw4, *_fDraw5;

    rw::core::Ptr< rwsim::drawable::RenderPoints > _xRender, _yRender, _zRender;
    rwlibs::drawable::Drawable *_xDraw, *_yDraw, *_zDraw;

    rw::core::Ptr< rwsim::drawable::RenderPoints > _selPosePntRenderX, _selPosePntRenderY,
        _selPosePntRenderZ;
    rwlibs::drawable::Drawable *_selPoseDrawX, *_selPoseDrawY, *_selPoseDrawZ;

    rw::core::Ptr< rwsim::drawable::RenderPoints > _selxRender, _selyRender, _selzRender;
    rwlibs::drawable::Drawable *_selxDraw, *_selyDraw, *_selzDraw;

    rw::core::Ptr< rwsim::drawable::RenderCircles > _xcRender, _ycRender, _zcRender;
    rwlibs::drawable::Drawable *_xcDraw, *_ycDraw, *_zcDraw;

    std::vector< rwsim::dynamics::RigidBody* > _bodies;
    std::vector< std::vector< rw::math::Vector3D<> > > _xaxis, _yaxis, _zaxis;
    std::map< rwsim::dynamics::RigidBody*, std::vector< rw::math::Vector3D<> > > _xaxisS, _yaxisS,
        _zaxisS;

    std::map< rwsim::dynamics::RigidBody*, std::vector< rwsim::util::SupportPose > > _supportPoses;
    std::map< rwsim::dynamics::RigidBody*, std::vector< rwsim::util::CircleModel > > _xcircBodyMap,
        _ycircBodyMap, _zcircBodyMap;
    std::map< rwsim::util::SupportPose*, std::vector< rwsim::util::CircleModel > > _xcirclesMap,
        _ycirclesMap, _zcirclesMap;
    std::map< std::pair< int, int >, std::vector< int > > _supportToPose;

    typedef std::vector< rw::math::Transform3D<> > PoseDistribution;
    std::map< rwsim::dynamics::RigidBody*, std::vector< PoseDistribution > >
        _supportPoseDistributions, _supportPoseDistributionsMisses;

    QGraphicsPixmapItem* _pitem;
    RestingPoseDialog* _restPoseDialog;

    rws::RobWorkStudio* _rwstudio;
};

#endif /* SupportPoseAnalyserDialog_HPP_ */
