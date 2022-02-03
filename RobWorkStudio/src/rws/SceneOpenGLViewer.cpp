/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SceneOpenGLViewer.hpp"

#include "ArcBallController.hpp"

#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/macros.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/graphics/Render.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/math/Constants.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>

#include <QCoreApplication>
#include <QMouseEvent>
#include <QThread>

using namespace rw::kinematics;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::core;
using namespace rw::geometry;
using namespace rwlibs::opengl;

using namespace rws;

namespace {

class RenderQuad : public Render
{
  private:
    GLfloat _colorTop[4], _colorBottom[4];
    int _x, _y, _width, _height;

  public:
    //! @brief smart pointer type to this class
    typedef rw::core::Ptr< RenderQuad > Ptr;

    /* Functions inherited from Render */
    /**
     * @copydoc Render::draw
     */
    void draw (const DrawableNode::RenderInfo& info, DrawType type, double alpha) const
    {
        glPushMatrix ();
        glLoadIdentity ();

        glPolygonMode (GL_FRONT, GL_FILL);
        glBegin (GL_QUADS);
        glColor4fv (_colorBottom);
        glVertex2f (_x, _y);
        glVertex2f (_width, _y);
        glColor4fv (_colorTop);
        glVertex2f (_width, _height);
        glVertex2f (_x, _height);
        glEnd ();

        glPopMatrix ();
    }

    void setTopColor (const Vector3D<>& color)
    {
        _colorTop[0] = color[0];
        _colorTop[1] = color[1];
        _colorTop[2] = color[2];
        _colorTop[3] = 1;
    }

    void setBottomColor (const Vector3D<>& color)
    {
        _colorBottom[0] = color[0];
        _colorBottom[1] = color[1];
        _colorBottom[2] = color[2];
        _colorBottom[3] = 1;
    }

    void setViewPort (int x, int y, int width, int height)
    {
        _x      = x;
        _y      = y;
        _width  = width;
        _height = height;
    }
};

/**
 * @brief clamp val to either min or max
 *
 * @param val [in] the value that is to be clamped
 * @param min [in] the minimum allowed value
 * @param max [in] the maximum allowed value
 * @return the clamped value of val
 */
template< class T > T clampI (T val, T min, T max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

QSurfaceFormat makeQSurfaceFormat (PropertyMap* map)
{
    QSurfaceFormat glf = QSurfaceFormat::defaultFormat ();

    if (map != NULL) {
        int nrSamples = map->add< int > ("GL_NR_SAMPLES", "", 4)->getValue ();
        if (map->add< bool > ("GL_MULTISAMPLE", "", false)->getValue ()) {
            nrSamples = clampI (nrSamples, 0, 8);
            glf.setSamples (nrSamples);
        }
    }
    return glf;
}

}    // namespace

void SceneOpenGLViewer::init ()
{
    using std::placeholders::_1;
    _pmap->getValue ()
        .add< bool > ("ReInitializeGL", "Reinitializes the opengl configuration.", false)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);

    _viewBackground = _pmap->getValue ().add< bool > ("DrawBackGround", "Draw Back Ground", true);
    _viewBackground->changedEvent ().add (
        std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1));

    _backgroundColorTop = _pmap->getValue ().add< Vector3D<> > (
        "BackGroundColorTop", "Top background color", Vector3D<> (1.0, 1.0, 1.0));
    _backgroundColorTop->changedEvent ().add (
        std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1));
    _backgroundColorBottom = _pmap->getValue ().add< Vector3D<> > (
        "BackGroundColorBottom", "Bottom background color", Vector3D<> (0.2, 0.2, 1.0));
    _backgroundColorBottom->changedEvent ().add (
        std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1));

    _pmap->getValue ()
        .add< bool > ("ShowCollisionModels", "Show Collision Models.", false)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);
    _pmap->getValue ()
        .add< bool > ("ShowVirtualModels", "Show Virtual Models.", true)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);
    _pmap->getValue ()
        .add< bool > ("ShowPhysicalModels", "Show Physical Models.", true)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);
    _pmap->getValue ()
        .add< bool > ("ShowDrawableModels", "Show Drawable Models.", true)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);
    _pmap->getValue ()
        .add< bool > ("ShowAllModels", "Show All Models.", false)
        ->changedEvent ()
        .add (std::bind (&SceneOpenGLViewer::propertyChangedListener, this, _1),
              this);

    int dmask = 0;
    if (_pmap->getValue ().get< bool > ("ShowCollisionModels", false))
        dmask |= DrawableNode::CollisionObject;
    if (_pmap->getValue ().get< bool > ("ShowVirtualModels", true))
        dmask |= DrawableNode::Virtual;
    if (_pmap->getValue ().get< bool > ("ShowPhysicalModels", true))
        dmask |= DrawableNode::Physical;
    if (_pmap->getValue ().get< bool > ("ShowDrawableModels", true))
        dmask |= DrawableNode::DrawableObject;
    if (_pmap->getValue ().get< bool > ("ShowAllModels", false))
        dmask |= DrawableNode::ALL;
    // this->setCheckForCollision(check);

    // add the default/main cameraview group
    _mainCamGroup = _scene->makeCameraGroup ("MainView");
    _scene->addCameraGroup (_mainCamGroup);
    _mainCamGroup->setEnabled (true);

    // add a node to render background
    rw::core::Ptr< RenderQuad > backgroundRender = ownedPtr (new RenderQuad ());

    backgroundRender->setTopColor (_backgroundColorTop->getValue ());
    backgroundRender->setBottomColor (_backgroundColorBottom->getValue ());
    backgroundRender->setViewPort (0, 0, 640, 480);
    _backgroundRender = backgroundRender;
    _backgroundnode =
        _scene->makeDrawable ("BackgroundRender", _backgroundRender, DrawableNode::ALL);
    _scene->addChild (_backgroundnode, _scene->getRoot ());

    _backgroundnode->setVisible (_viewBackground->getValue ());

    _worldNode = _scene->makeGroupNode ("World");
    _scene->addChild (_worldNode, _scene->getRoot ());

    _mainView            = ownedPtr (new SceneViewer::View ("MainView"));
    _mainView->_drawMask = dmask;
    _currentView         = _mainView;
    // add background camera
    _backCam = _scene->makeCamera ("BackgroundCam");
    _backCam->setEnabled (true);
    _backCam->setRefNode (_backgroundnode);
    _backCam->setProjectionMatrix (ProjectionMatrix::makeOrtho (0, 640, 0, 480, -1, 1));
    _backCam->setClearBufferEnabled (true);
    _backCam->setClearBufferMask (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _backCam->setDepthTestEnabled (false);
    _backCam->setLightningEnabled (false);
    _mainCamGroup->insertCamera (_backCam, 0);

    // main camera
    _mainCam = _scene->makeCamera ("MainCam");
    _mainCam->setDrawMask (dmask);
    _mainCam->setEnabled (false);
    _mainCam->setPerspective (45, 640, 480, 0.01, 30);
    _mainCam->setClearBufferEnabled (false);
    _mainCam->setClearBufferMask (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _mainCam->setDepthTestEnabled (true);
    _mainCam->setLightningEnabled (true);
    _mainCam->setRefNode (_scene->getRoot ());
    _mainCamGroup->insertCamera (_mainCam, 1);
    // TODO: foreground camera
    _mainView->_viewCamera = _mainCam;
    _mainView->_camGroup   = _mainCamGroup;
    _pivotDrawable         = NULL;

    this->setFocusPolicy (Qt::StrongFocus);
}

SceneViewer::View::Ptr SceneOpenGLViewer::createView (const std::string& name,
                                                      bool enableBackground)
{
    SceneViewer::View::Ptr nview = ownedPtr (new SceneViewer::View (name));
    nview->_viewCamera           = _scene->makeCamera ("ViewCamera");
    nview->_camGroup             = _scene->makeCameraGroup ("ViewCamera");

    if (enableBackground) {
        SceneCamera::Ptr backCam = _scene->makeCamera ("BackgroundCam");
        backCam->setEnabled (true);
        backCam->setRefNode (_backgroundnode);
        backCam->setProjectionMatrix (ProjectionMatrix::makeOrtho (0, 640, 0, 480, -1, 1));
        backCam->setClearBufferEnabled (true);
        backCam->setClearBufferMask (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        backCam->setDepthTestEnabled (false);
        backCam->setLightningEnabled (false);
        nview->_camGroup->insertCamera (backCam, 0);
    }

    nview->_viewCamera->setEnabled (true);
    nview->_viewCamera->setPerspective (45, 640, 480, 0.1, 30);
    if (enableBackground) {
        nview->_viewCamera->setClearBufferEnabled (false);
    }
    else {
        nview->_viewCamera->setClearBufferEnabled (true);
    }
    nview->_viewCamera->setClearBufferMask (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    nview->_viewCamera->setDepthTestEnabled (true);
    nview->_viewCamera->setLightningEnabled (true);
    nview->_viewCamera->setRefNode (_scene->getRoot ());
    nview->_viewCamera->setViewport (0, 0, _width, _height);
    nview->_camGroup->insertCamera (nview->_viewCamera, 1);

    nview->_camGroup->setEnabled (true);
    nview->_viewCamera->setAspectRatioControl (SceneCamera::Scale);
    _views.push_back (nview);
    return nview;
}

SceneOpenGLViewer::SceneOpenGLViewer (QWidget* parent) :
    QOpenGLWidget (parent), _scene (ownedPtr (new SceneOpenGL ())),
    _viewLogo ("RobWork")
{
    setFormat(makeQSurfaceFormat (NULL));
    // start by initializing propertymap
    _pmap = ownedPtr (new Property< PropertyMap > ("SceneViewer", "", PropertyMap ()));
    _pmap->getValue ().add< int > ("GL_NR_SAMPLES", "", 4);
    _pmap->getValue ().add< bool > ("GL_MULTISAMPLE", "", false);
    init ();
    setAcceptDrops (true);
    _cameraCtrl = ownedPtr (new ArcBallController (640, 480, _mainCam));
}

SceneOpenGLViewer::SceneOpenGLViewer (PropertyMap& pmap, QWidget* parent) :
    QOpenGLWidget (parent),
    _scene (ownedPtr (new SceneOpenGL ())), _viewLogo ("RobWork")
{
    setFormat(makeQSurfaceFormat (pmap.getPtr< PropertyMap > ("SceneViewer")));
    // start by initializing propertymap
    _pmap = pmap.add< PropertyMap > ("SceneViewer", "", PropertyMap ());

    init ();
    setAcceptDrops (true);
    _cameraCtrl = ownedPtr (new ArcBallController (640, 480, _mainCam));
}

SceneOpenGLViewer::~SceneOpenGLViewer ()
{}

void SceneOpenGLViewer::setWorldNode (rw::graphics::GroupNode::Ptr wnode)
{
    RW_ASSERT (wnode != NULL);

    if (_pivotDrawable == NULL) {
        /// TODO: this should be simplified to orthographic camera view. And only drawn in 2D
        _pivotDrawable =
            _scene->makeDrawable ("Pivot", Geometry::makeSphere (0.01), DrawableNode::Virtual);
        _scene->addChild (_pivotDrawable, _scene->getRoot ());
        _pivotDrawable->setColor (Vector3D<> (1.0f, 0.0f, 0.0f));
        _cameraCtrl->setDrawable (_pivotDrawable);
    }

    if (wnode == NULL) {
        _mainCam->setEnabled (false);
    }
    else {
        _mainCam->setEnabled (true);
    }

    // reattach the pivot drawable
    _scene->removeDrawable (_pivotDrawable);
    _scene->getRoot ()->removeChild (_worldNode);
    _worldNode = wnode;
    if (!_scene->getRoot ()->hasChild (_worldNode)) {
        _scene->addChild (_worldNode, _scene->getRoot ());
    }
    _scene->addChild (_pivotDrawable, _worldNode);

    _mainCam->setRefNode (_worldNode);
    RW_ASSERT (_worldNode->hasChild (_pivotDrawable) == true);
    RW_ASSERT (_pivotDrawable->hasParent (_worldNode) == true);
}

void SceneOpenGLViewer::keyPressEvent (QKeyEvent* e)
{
    e->ignore ();
    QOpenGLWidget::keyPressEvent (e);
}

void SceneOpenGLViewer::clear ()
{
    // reset everything
}

void SceneOpenGLViewer::renderView (View::Ptr view)
{
    const bool isGuiThread = QThread::currentThread () == QCoreApplication::instance ()->thread ();
    if (isGuiThread) {
        renderViewThreadSafe (view);
    }
    else {
        qRegisterMetaType< View::Ptr > ("View::Ptr");
        QMetaObject::invokeMethod (
            this, "renderViewThreadSafe", Qt::BlockingQueuedConnection, Q_ARG (View::Ptr, view));
    }
}

void SceneOpenGLViewer::renderViewThreadSafe (View::Ptr view)
{
    // boost::mutex::scoped_lock lock(_renderMutex);
    makeCurrent ();

    _renderInfo._mask     = view->_drawMask;
    _renderInfo._drawType = view->_drawType;
    _renderInfo.cams      = view->_camGroup;

    _scene->draw (_renderInfo);

    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }
    doneCurrent();
}

void SceneOpenGLViewer::initializeGL ()
{
    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }
    /****************************************/
    /* Set up OpenGL lights etc.            */
    /****************************************/
    if (_pmap->getValue ().add< bool > ("GL_DEPTH_TEST", "Enable depth testing", true)) {
        glEnable (GL_DEPTH_TEST);
        std::string desc ("Possible values: GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, GL_GREATER, "
                          "GL_NOTEQUAL, GL_GEQUAL, GL_ALWAYS");
        int val = _pmap->getValue ().add< int > ("glDepthFunc", desc, GL_LESS)->getValue ();
        glDepthFunc (val);    // GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, GL_GREATER, GL_NOTEQUAL,
                              // GL_GEQUAL, GL_ALWAYS
    }
    else {
        glDisable (GL_DEPTH_TEST);
    }

    int val = _pmap->getValue ().add< int > ("glShadeModel", "", GL_SMOOTH)->getValue ();
    glShadeModel (val);    // GL_FLAT, GL_SMOOTH

    // glDisable( GL_COLOR_MATERIAL );
    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }

    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // glEnable(GL_TEXTURE_2D);
    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }

    if (_pmap->getValue ().add< bool > ("GL_LIGHTING", "", true)->getValue ())
        glEnable (GL_LIGHTING);
    if (_pmap->getValue ().add< bool > ("GL_NORMALIZE", "", true)->getValue ())
        glEnable (GL_NORMALIZE);
    if (_pmap->getValue ().add< bool > ("GL_DEPTH_TEST", "", true)->getValue ())
        glEnable (GL_DEPTH_TEST);

    GLfloat light0_ambient[]  = {0.1f, 0.1f, 0.1f, 1.0f};
    GLfloat light0_diffuse[]  = {.8f, .8f, 0.8f, 1.0f};
    GLfloat light0_specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat light0_position[] = {0.0f, 0.0f, 1.0f, 0.0f};    // point light, from above

    // GLfloat light0_ambient[] =  {0.0f, 0.0f, 0.0f, 1.0f};
    // GLfloat light0_diffuse[] =  {1.0f, 1.0f, 1.0f, 1.0f};
    // GLfloat light0_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    // GLfloat light0_position[] = {0.0f, 0.0f, 1.0f, 0.0f}; // point light, from above

    glLightfv (GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv (GL_LIGHT0, GL_SPECULAR, light0_specular);
    glLightfv (GL_LIGHT0, GL_POSITION, light0_position);

    GLfloat global_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, global_ambient);

    glEnable (GL_LIGHT0);
    DrawableUtil::setupHighlightLight ();
    glEnable (GL_COLOR_MATERIAL);
    GLenum matRendering = GL_FRONT_AND_BACK;
    // GLenum matRendering = GL_FRONT;
    glColorMaterial (matRendering, GL_AMBIENT_AND_DIFFUSE);

    GLfloat specularReflection[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat matEmission[]        = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv (matRendering, GL_SPECULAR, specularReflection);
    glMaterialfv (matRendering, GL_EMISSION, matEmission);
    glMateriali (matRendering, GL_SHININESS, 128);

    // glEnable(GL_COLOR_MATERIAL);
    // glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    if (_pmap->getValue ().add< bool > ("GL_LINE_SMOOTH", "", true)->getValue ()) {
        glEnable (GL_LINE_SMOOTH);
        glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);    // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    }
    else {
        glDisable (GL_LINE_SMOOTH);
    }

    if (_pmap->getValue ().add< bool > ("GL_POINT_SMOOTH", "", false)->getValue ()) {
        glEnable (GL_POINT_SMOOTH);
        glHint (GL_POINT_SMOOTH_HINT, GL_NICEST);    // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    }
    else {
        glDisable (GL_POINT_SMOOTH);
    }

    if (_pmap->getValue ().add< bool > ("GL_POLYGON_SMOOTH", "", false)->getValue ()) {
        glEnable (GL_POLYGON_SMOOTH);
        glHint (GL_POLYGON_SMOOTH_HINT, GL_NICEST);    // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    }
    else {
        glDisable (GL_POLYGON_SMOOTH);
    }

    // if( _pmap->getValue().add<bool>("GL_PERSPECTIVE_CORRECTION","",false)->getValue() ){
    //    glEnable(GL_PERSPECTIVE_CORRECTION);
    //    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    //} else {
    //    glDisable(GL_PERSPECTIVE_CORRECTION);
    //}

    if (_pmap->getValue ().add< bool > ("GL_BLEND", "", true)->getValue ()) {
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);
        // glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
    }
    else {
        glDisable (GL_BLEND);
    }

    if (_pmap->getValue ().add< bool > ("GL_ALHPA_TEST", "", false)->getValue ()) {
        glEnable (GL_ALPHA_TEST);          // allows alpha channels or transperancy
        glAlphaFunc (GL_GREATER, 0.1f);    // sets aplha function
    }
    else {
        glDisable (GL_ALPHA_TEST);
    }
    glClearColor (1.0f, 1.0f, 1.0f, 1.0f);
    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }
}

void SceneOpenGLViewer::paintGL ()
{
    if (_cameraCtrl) {
        // for now we scale the pivot such that its not unrealistically large/small
        getViewCamera ()->setTransform (_cameraCtrl->getTransform ());
    }

    _renderInfo._drawType = _currentView->_drawType;
    _renderInfo._mask     = _currentView->_drawMask;
    _renderInfo.cams      = _currentView->_camGroup;
    _scene->draw (_renderInfo);

    {
        const std::string error = SceneOpenGL::detectGLerror ();
        if (!error.empty ())
            RW_WARN ("OpenGL error detected:" << error);
    }
}

void SceneOpenGLViewer::resizeGL (int width, int height)
{
    _width  = width;
    _height = height;
    for (SceneCamera::Ptr cam : _currentView->_camGroup->getCameras ()) {
        cam->setViewport (0, 0, _width, _height);
    }

    ((RenderQuad*) _backgroundRender.get ())->setViewPort (0, 0, _width, _height);
    _cameraCtrl->setBounds (width, height);
}

void SceneOpenGLViewer::selectView (View::Ptr view)
{
    _currentView = view;
    for (SceneCamera::Ptr cam : _currentView->_camGroup->getCameras ()) {
        cam->setViewport (0, 0, _width, _height);
    }
}

void SceneOpenGLViewer::destroyView (View::Ptr view)
{
    // TODO: remove camera from scene
    if (_mainView == view) {
        RW_THROW ("The View \"" << view->_name << "\" is the MainView, which cannot be removed!");
    }

    if (_currentView == view || _mainView == view) {
        RW_THROW ("The View \"" << view->_name
                                << "\" is Active. Please select another view before removing it!");
    }

    std::vector< View::Ptr > nviews;
    for (View::Ptr v : _views) {
        if (v != view)
            nviews.push_back (v);
    }
    _views = nviews;

    _scene->removeCameraGroup (view->_camGroup);
}

void SceneOpenGLViewer::updateState (const State& state)
{
    if (_state == NULL)
        _state = rw::core::ownedPtr (new State ());
    *_state            = state;
    _renderInfo._state = _state.get ();
}

DrawableNode::Ptr SceneOpenGLViewer::pickDrawable (int x, int y)
{
    makeCurrent();
    return _scene->pickDrawable (_renderInfo, x, y);
    doneCurrent();
}

DrawableNode::Ptr SceneOpenGLViewer::pickDrawable (rw::graphics::SceneGraph::RenderInfo& info,
                                                   int x, int y)
{
    makeCurrent();
    return _scene->pickDrawable (info, x, y);
    doneCurrent();
}

rw::kinematics::Frame* SceneOpenGLViewer::pickFrame (int x, int y)
{
    DrawableNode::Ptr d = pickDrawable (x, y);
    if (d == NULL) {
        return NULL;
    }
    Frame* res = (_wcscene) ? _wcscene->getFrame (d) : NULL;
    if (res == NULL) {
        return NULL;
    }
    std::cout << "Frame: " << res->getName () << std::endl;
    return res;
}

void SceneOpenGLViewer::mouseDoubleClickEvent (QMouseEvent* event)
{
    if (event->button () == Qt::LeftButton) {
        int winx = event->pos().x();
        int winy = height () - event->pos().y();

        makeCurrent();
        Vector3D<> pos = _scene->unproject (_mainCam, winx, winy);
        doneCurrent();

        if (pos[2] != 1) {
            // double click + SHIFT => positionSelected event
            if (event->modifiers () == Qt::ShiftModifier) {
                positionSelectedEvent ().fire (pos);

                // doubleclick + CONTROL => frameSelected event
            }
            else if (event->modifiers () == Qt::ControlModifier) {
                Frame* frame = pickFrame (winx, winy);
                if (frame) {
                    // frameSelectedEvent().fire(frame);
                }

                // plain doubleclick => move pivot point
            }
            else {
                _cameraCtrl->setCenter (pos, Vector2D<> (event->pos().x(), event->pos().y()));
                _pivotDrawable->setTransform (Transform3D<> (pos, Rotation3D<>::identity ()));
                QWidget::update ();
            }
        }
    }
    else {
        event->ignore ();
    }
    // std::cout << "forward double click" << std::endl;
    QOpenGLWidget::mouseDoubleClickEvent (event);
}

void SceneOpenGLViewer::mousePressEvent (QMouseEvent* event)
{
    QMouseEvent* e = static_cast< QMouseEvent* > (event);
    if (e->buttons () == Qt::RightButton) {
        makeCurrent();
        Vector3D<> pos = _scene->unproject (_mainCam, e->pos().x(), height () - e->pos().y());
        doneCurrent();
        _cameraCtrl->handleEvent (event);
        _cameraCtrl->setPanTarget (pos);
    }
    else {
        _cameraCtrl->handleEvent (event);
    }

    QOpenGLWidget::mousePressEvent (event);
}

void SceneOpenGLViewer::mouseMoveEvent (QMouseEvent* event)
{
    _cameraCtrl->handleEvent (event);

    // std::cout<<"Event Time"<<eventTimer.getTime()<<std::endl;
    QWidget::update ();

    // event->ignore();
    QOpenGLWidget::mouseMoveEvent (event);
}

void SceneOpenGLViewer::wheelEvent (QWheelEvent* event)
{
#if QT_VERSION >= 0x050E00
    int winx       = event->position().x();
    int winy       = height () - event->position().y();
#else
    int winx       = event->x ();
    int winy       = height () - event->y ();
#endif
    makeCurrent();
    Vector3D<> pos = _scene->unproject (_mainCam, winx, winy);
    doneCurrent();
    _cameraCtrl->setZoomTarget (pos);
    _cameraCtrl->handleEvent (event);
    QWidget::update ();
    QOpenGLWidget::wheelEvent (event);
}

void SceneOpenGLViewer::saveBufferToFile (const std::string& stdfilename, const int fillR,
                                          const int fillG, const int fillB)
{
    QString filename (stdfilename.c_str ());
    QImage img = grabFramebuffer ();
    if (_currentView->_viewCamera->getAspectRatioControl () == SceneCamera::Fixed) {
        int x, y, w, h;
        _currentView->_viewCamera->getViewport (x, y, w, h);

        // Get height of grabbed image
        const int width = img.size ().rwidth (), height = img.size ().rheight ();

        // Instantiate result
        QImage dstimg;
        if (height >= h) {    // If the image is taller than the camera view port
            // Move down to where the image starts and copy
            dstimg = img.copy (0, height - h, w, h);
        }
        else {    // Else
            // Fill
            dstimg = QImage (w, h, img.format ());
            dstimg.fill (qRgb (fillR, fillG, fillB));
            // Insert at bottom of destination
            const int yOffset = h - height;
            for (int x = 0; x < std::min (width, w); ++x) {
                for (int y = yOffset; y < h; ++y) {
                    dstimg.setPixel (x, y, img.pixel (x, y - yOffset));
                }
            }
        }

        img = dstimg;
    }

    if (!img.save (filename))
        throw std::string ("SceneOpenGLViewer::saveBufferToFile: Could not save file: ") +
            filename.toStdString ();
}

void SceneOpenGLViewer::propertyChangedListener (PropertyBase* base)
{
    std::string id = base->getIdentifier ();
    // std::cout << "Property Changed Listerner SceneOpenGLViewer: " << id << std::endl;
    if (id == "ShowCollisionModels") {
        Property< bool >* p = toProperty< bool > (base);
        int dmask           = getViewCamera ()->getDrawMask ();
        if (p == NULL)
            return;
        if (p->getValue ()) {
            dmask = dmask | DrawableNode::CollisionObject;
        }
        else {
            dmask = dmask & ~DrawableNode::CollisionObject;
        }
        getViewCamera ()->setDrawMask (dmask);
    }
    else if (id == "ShowVirtualModels") {
        Property< bool >* p = toProperty< bool > (base);
        int dmask           = getViewCamera ()->getDrawMask ();
        if (p == NULL)
            return;
        if (p->getValue ()) {
            dmask = dmask | DrawableNode::Virtual;
        }
        else {
            dmask = dmask & ~DrawableNode::Virtual;
        }
        getViewCamera ()->setDrawMask (dmask);
    }
    else if (id == "ShowPhysicalModels") {
        Property< bool >* p = toProperty< bool > (base);
        int dmask           = getViewCamera ()->getDrawMask ();
        if (p == NULL)
            return;
        if (p->getValue ()) {
            dmask = dmask | DrawableNode::Physical;
        }
        else {
            dmask = dmask & ~DrawableNode::Physical;
        }
        getViewCamera ()->setDrawMask (dmask);
    }
    else if (id == "ShowDrawableModels") {
        Property< bool >* p = toProperty< bool > (base);
        int dmask           = getViewCamera ()->getDrawMask ();
        if (p == NULL)
            return;
        if (p->getValue ()) {
            dmask = dmask | DrawableNode::DrawableObject;
        }
        else {
            dmask = dmask & ~DrawableNode::DrawableObject;
        }
        getViewCamera ()->setDrawMask (dmask);
    }
    else if (id == "ShowAllModels") {
        Property< bool >* p = toProperty< bool > (base);
        int dmask           = getViewCamera ()->getDrawMask ();
        if (p != NULL) {
            if (p->getValue ()) {
                dmask = dmask | DrawableNode::ALL;
            }
            else {
                int dmask = 0;
                if (_pmap->getValue ().get< bool > ("ShowCollisionModels", false))
                    dmask |= DrawableNode::CollisionObject;
                if (_pmap->getValue ().get< bool > ("ShowVirtualModels", true))
                    dmask |= DrawableNode::Virtual;
                if (_pmap->getValue ().get< bool > ("ShowPhysicalModels", true))
                    dmask |= DrawableNode::Physical;
                if (_pmap->getValue ().get< bool > ("ShowDrawableModels", true))
                    dmask |= DrawableNode::DrawableObject;
            }
            getViewCamera ()->setDrawMask (dmask);
        }
    }
    else if (base == _viewBackground) {
        _backgroundnode->setVisible (_viewBackground->getValue ());
    }
    else if (id == "BackGroundColorBottom") {
        _backgroundRender.cast< RenderQuad > ()->setBottomColor (
            _backgroundColorBottom->getValue ());
    }
    else if (id == "BackGroundColorTop") {
        _backgroundRender.cast< RenderQuad > ()->setBottomColor (_backgroundColorTop->getValue ());
    }
    else if (id == "DrawWorldGrid") {
    }
    else if (id == "ReInitializeGL") {
    }
}
void SceneOpenGLViewer::zoom (double amount)
{
    _cameraCtrl->zoom (amount);
}

void SceneOpenGLViewer::autoZoom ()
{
    if (!(_wcscene->getWorkCell ())) {
        RW_WARN ("Can't autozoom when no workcell is loaded");
        return;
    }
    static const double fovy = 45. * Deg2Rad;
    int x, y, width, height;
    _mainCam->getViewport (x, y, width, height);
    const double aspectRatio = static_cast< double > (width) / static_cast< double > (height);
    _cameraCtrl->autoZoom (_wcscene->getWorkCell (), _state, fovy, aspectRatio);
}
