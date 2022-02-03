/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "XmlCalibrationLoader.hpp"

#include <rw/core/DOMElem.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwlibs/calibration/FixedFrameCalibration.hpp>

using namespace rwlibs::calibration;
using namespace rw::core;
using namespace rw::math;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw;

namespace {

FixedFrameCalibration::Ptr readFixedFrameCalibration (DOMElem::Ptr felement, WorkCell::Ptr workcell)
{
    DOMElem::Ptr element = felement->getChild ("FixedFrameCalibration");

    if (!element->hasAttribute ("frame"))
        RW_THROW ("\"frame\" attribute missing.");
    std::string frameName        = element->getAttributeValue ("frame");
    rw::kinematics::Frame* frame = workcell->findFrame (frameName);
    rw::kinematics::FixedFrame::Ptr fixedFrame =
        rw::kinematics::Frame::Ptr (frame).cast< rw::kinematics::FixedFrame > ();
    if (fixedFrame.isNull ())
        RW_THROW ("Frame \"" << frameName << "\" not found.");

    DOMElem::Ptr transformElement = element->getChild ("Transform3D");
    if (transformElement == NULL)
        RW_THROW ("\"Transform3D\" element not found");

    Transform3D<> t3d = DOMBasisTypes::readTransform3D (transformElement, false);

    // if (!transformElement->hasAttribute("isPostCorrection"))
    //	RW_THROW("\"isPostCorrection\" attribute missing.");
    // bool isPostCorrection = transformElement->getAttributeValueAsBool("isPostCorrection");

    return rw::core::ownedPtr (new FixedFrameCalibration (fixedFrame, t3d));
}

}    // namespace

WorkCellCalibration::Ptr XmlCalibrationLoader::load (rw::models::WorkCell::Ptr workcell,
                                                     std::string fileName)
{
    DOMParser::Ptr parser = DOMParser::make ();

    parser->load (fileName);

    DOMElem::Ptr elmRoot = parser->getRootElement ();

    if (!elmRoot->hasChild ("WorkCellCalibration"))
        RW_THROW ("Element not found.");
    elmRoot                                      = elmRoot->getChild ("WorkCellCalibration");
    WorkCellCalibration::Ptr workcellCalibration = ownedPtr (new WorkCellCalibration ());
    for (DOMElem::Ptr child : elmRoot->getChildren ()) {
        if (child->getName () == "FixedFrameCalibration") {
            FixedFrameCalibration::Ptr ffCalibration = readFixedFrameCalibration (child, workcell);
            workcellCalibration->addCalibration (ffCalibration);
        }
    }

    return workcellCalibration;
}
