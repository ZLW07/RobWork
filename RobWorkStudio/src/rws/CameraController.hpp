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

#ifndef RWS_CAMERACONTROLLER_HPP_
#define RWS_CAMERACONTROLLER_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

#include <QEvent>

namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics
namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models

namespace rws {

/**
 * @brief an interface for controlling the camera using a mouse.
 */
class CameraController
{
  public:
    //! @brief smart pointer type of this class
    typedef rw::core::Ptr< CameraController > Ptr;

    //! @brief destructor
    virtual ~CameraController () {}

    /**
     * @brief set the bounds that define the area where the 2d point
     * is valid. The bound is defined in a plane with [0,width] and
     * [0,height]. Where (0,0) is the upper left corner of the plane.
     * @param NewWidth [in] width
     * @param NewHeight [in] height
     */
    virtual void setBounds (double NewWidth, double NewHeight) = 0;

    /**
     * @brief update the center of rotation and screen center
     * @param center [in] center of rotation in world coordinates
     * @param screenCenter [in] center of rotation in screen coordinates
     */
    virtual void setCenter (const rw::math::Vector3D<>& center,
                            const rw::math::Vector2D<>& screenCenter) = 0;

    /**
     * @brief event handler, typically mouse and keyboard
     * @param event [in] the specific event
     */
    virtual void handleEvent (QEvent* event) = 0;

    /**
     * @brief set world to camera transformation
     * @param t3d [in] world to camera transformation
     */
    virtual void setTransform (const rw::math::Transform3D<>& t3d) = 0;

    /**
     * @brief get the current world to camera transformation
     * @return world to camera transformation
     */
    virtual rw::math::Transform3D<> getTransform () const = 0;

    /**
     * @brief get the current pivot point in world coordinates
     * @return current pivot point
     */
    virtual rw::math::Vector3D<> getCenter () = 0;

    /**
     * @brief draw the camera control.
     */
    virtual void draw () = 0;

    /**
     * @brief Zoom by amount specified by amount.
     *
     * Calling this method moves the camera along its Z-axis.
     * @param amount [in] Meters to zoom the camera
     */
    virtual void zoom (double amount) = 0;

    /**
     * @brief Zooms the camera to fit all devices into the camera view.
     *
     * Useful when working with robots smaller or larger than standard.
     * Calling this method moves the camera along its Z-axis.
     * @param workcell [in] The autozoom functions fits all frames of workcell in the viewport
     * @param state [in] state with the current positions of the frames. If NULL, the default
     * workcell state is used.
     * @param fovy [in] the field of view in the vertical direction (in radians).
     * @param aspectRatio [in] the aspect ratio of (width divided by height).
     */
    virtual void autoZoom (rw::core::Ptr< rw::models::WorkCell > workcell,
                           rw::core::Ptr< const rw::kinematics::State > state, double fovy,
                           double aspectRatio) = 0;

    /**
     * @brief set the 3D coordinate that should be zoomed towards.
     * @param target [in] the target position to zoom towards.
     * @param enable [in] enable or disable zoom with zoomtarget.
     */
    virtual void setZoomTarget (rw::math::Vector3D< double > target, bool enable = true){};

    /**
     * @brief set the 3D coordinate that should be moved.
     * @param target [in] the target position to move.
     * @param enable [in] enable or disable pan with pantarget.
     */
    virtual void setPanTarget (rw::math::Vector3D< double > target, bool enable = true){};

    /**
     * @brief get the 3D coordinate that should be moved.
     * @return the target position to move.
     */
    virtual rw::math::Vector3D<> getPanTarget () { return rw::math::Vector3D<> (); };

    /**
     * @brief add a drawable to the camera controller
     * @param obj [in] a drawable to be controled by the camera controller
     */
    virtual void setDrawable (rw::graphics::DrawableNode::Ptr obj){};
};

}    // namespace rws

#endif /* CAMERACONTROLLER_HPP_ */
