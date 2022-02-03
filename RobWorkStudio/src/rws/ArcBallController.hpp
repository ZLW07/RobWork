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

#ifndef RWS_ArcBallController_HPP
#define RWS_ArcBallController_HPP

#include "CameraController.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/graphics/SceneCamera.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rws {

/**
 * @brief Use the ArcBallController method to control the camera view point in a scene.
 *
 * The ArcBallController method
 * defines a way to map a 2d position into a 3d position on some sphere.
 * This means that the rotation is defined by dragging a point on a sphere
 * around its center.
 */
class ArcBallController : public CameraController
{
  protected:
    /**
     * @brief maps a 2d position (x,y) into a position on a 3D sphere
     * which is centered around \b _centerPt.
     * @param x [in]
     * @param y [in]
     */
    rw::math::Vector3D<> mapToSphere (double x, double y) const;

    /**
     * @brief project a image position into 3D space using pinhole model
     * @param x [in] the x coordinate in the image
     * @param y [in] the y coordinate in the image
     * @param z [in] the z coordinate of the world
     *
     * @return the projected 3D position
     */
    rw::math::Vector3D<> unproject (int x, int y, double z);

    /**
     * @brief move the camera in 2D relative to the scene
     *
     * @param x [in] current mouse position in x-axis
     * @param y [in] current mouse position in y-axis
     */
    void pan (int x, int y);

  public:
    //! @brief Smart pointer type for ArcBallController
    typedef rw::core::Ptr< ArcBallController > Ptr;

    /**
     * @brief constructor
     */
    ArcBallController (double NewWidth, double NewHeight);

    /**
     * @brief constructor
     */
    ArcBallController (double NewWidth, double NewHeight, rw::graphics::SceneCamera::Ptr cam);

    /**
     * @brief destructor
     */
    virtual ~ArcBallController ()
    { /* nothing to do */
    }

    /**
     * @brief register a mouse click event. The coordinates must be inside the
     * specified bounds.
     * @param x [in] x-coodinate
     * @param y [in] y-coodinate
     */
    void click (float x, float y);

    /**
     * @brief Calculates the rotation of the object/scene based on
     * the mouse being dragged to the position (x,y).
     * @param x [in] the x-coordinate of the current mouse position
     * @param y [in] the y-coordinate of the current mouse position
     * @return the rotation that should be applied to the object/scene
     */
    rw::math::Quaternion< double > drag (float x, float y);

    //! @copydoc CameraController::draw
    void draw ();

    //! @copydoc CameraController::setBounds
    void setBounds (double NewWidth, double NewHeight);

    //! @copydoc CameraController::handleEvent
    virtual void handleEvent (QEvent* event);

    //! @copydoc CameraController::getTransform
    virtual rw::math::Transform3D<> getTransform () const;

    //! @copydoc CameraController::setTransform
    void setTransform (const rw::math::Transform3D<>& t3d);

    //! @copydoc CameraController::setCenter
    void setCenter (const rw::math::Vector3D<>& center, const rw::math::Vector2D<>& screenCenter);

    //! @copydoc CameraController::getCenter
    rw::math::Vector3D<> getCenter () { return _pivotPoint; }

    //! @copydoc CameraController::zoom
    void zoom (double amount);

    //! @copydoc CameraController::autoZoom
    void autoZoom (rw::core::Ptr< rw::models::WorkCell > workcell,
                   rw::core::Ptr< const rw::kinematics::State > state, double fovy,
                   double aspectRatio);

    //! @copydoc CameraController::setZoomTarget
    void setZoomTarget (rw::math::Vector3D< double > target, bool enable = true);

    //! @copydoc CameraController::setPanTarget
    void setPanTarget (rw::math::Vector3D< double > target, bool enable = true);

    //! @copydoc CameraController::getPanTarget
    rw::math::Vector3D<> getPanTarget ();

    //! @copydoc CameraController::setDrawable
    void setDrawable (rw::graphics::DrawableNode::Ptr obj) { _pivotObj = obj; }

  private:
    rw::math::Vector2D<> _centerPt;    // Center of the ball
    rw::math::Vector3D<> _stVec;       // Saved click vector
    rw::math::Vector3D<> _enVec;       // Saved drag vector
    double _adjustWidth;               // Mouse bounds width
    double _adjustHeight;              // Mouse bounds height
    double _height, _width;
    rw::math::Transform3D<> _viewTransform;
    // rw::math::Rotation3D<> _viewRotation;
    rw::math::Vector3D<> _lastPos, _pivotPoint;
    rw::math::Transform3D<> _camTransform;

    bool _advancedZoomEnabled;
    rw::math::Vector3D< double > _zoomTarget;

    bool _advancedPanEnabled;
    rw::math::Vector3D<> _initialPanPos;
    rw::math::Transform3D<> _initial_cTw;
    double _initialZ;

    rw::graphics::DrawableNode::Ptr _pivotObj;

    rw::graphics::SceneCamera::Ptr _cam;

    void setPivotScale ();
};

}    // namespace rws

#endif
