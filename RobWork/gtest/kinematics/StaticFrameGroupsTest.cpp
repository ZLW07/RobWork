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

#include <rw/kinematics.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/WorkCell.hpp>


#include <gtest/gtest.h>
#include <iostream>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;

namespace {
void addTestFrames (WorkCell& world)
{
    MovableFrame::Ptr frame1  = ownedPtr(new MovableFrame ("Frame1"));
    MovableFrame::Ptr frame2  = ownedPtr(new MovableFrame ("Frame2"));
    MovableFrame::Ptr frame3  = ownedPtr(new MovableFrame ("Frame3"));
    FixedFrame::Ptr frame4    = ownedPtr(new FixedFrame ("Frame4", Transform3D< double > ()));
    FixedFrame::Ptr frame5    = ownedPtr(new FixedFrame ("Frame5", Transform3D< double > ()));
    MovableFrame::Ptr frame6  = ownedPtr(new MovableFrame ("Frame6"));
    MovableFrame::Ptr frame7  = ownedPtr(new MovableFrame ("Frame7"));
    MovableFrame::Ptr frame8  = ownedPtr(new MovableFrame ("Frame8"));
    MovableFrame::Ptr frame9  = ownedPtr(new MovableFrame ("Frame9"));
    MovableFrame::Ptr frame10 = ownedPtr(new MovableFrame ("Frame10"));
    MovableFrame::Ptr frame11 = ownedPtr(new MovableFrame ("Frame11"));
    FixedFrame::Ptr frame12   = ownedPtr(new FixedFrame ("Frame12", Transform3D< double > ()));

    world.addFrame (frame1);
    world.addFrame (frame3, frame1);
    world.addFrame (frame4, frame1);
    world.addDAF (frame8, frame1);
    world.addDAF (frame7, frame4);

    world.addFrame (frame2);
    world.addFrame (frame5, frame2);
    world.addFrame (frame6, frame5);
    world.addFrame (frame9, frame6);
    world.addDAF (frame10, frame9);
    world.addDAF (frame11, frame9);

    world.addFrame (frame12);
}
}    // namespace

TEST (Kinematics, getStaticFrameGroups)
{
    WorkCell world ("The World");
    addTestFrames (world);

    std::vector< bool > frameInGroup (13, false);

    Frame* const root = world.getWorldFrame ();
    std::vector< FrameList > staticGroups =
        Kinematics::getStaticFrameGroups (root, world.getDefaultState ());
    EXPECT_EQ (10u, staticGroups.size ());
    for (FrameList& list : staticGroups) {
        for (const Frame* frame : list) {
            if (frame->getName () == "WORLD") {
                EXPECT_FALSE (frameInGroup[0]);
                frameInGroup[0] = true;
            }
            else {
                const int id = std::stoi (frame->getName ().substr (5));
                EXPECT_FALSE (frameInGroup[id]);
                frameInGroup[id] = true;
            }
        }
    }
    for (std::size_t i = 0; i < frameInGroup.size (); i++) {
        EXPECT_TRUE (frameInGroup[i]);
    }
}

TEST (Kinematics, getStaticFrameGroupsConst)
{
    WorkCell world ("The World");
    addTestFrames (world);

    std::vector< bool > frameInGroup (13, false);

    const Frame* const root = world.getWorldFrame ();
    std::vector< ConstFrameList > staticGroups =
        Kinematics::getStaticFrameGroups (root, world.getDefaultState ());
    EXPECT_EQ (10u, staticGroups.size ());
    for (ConstFrameList& list : staticGroups) {
        for (const Frame* frame : list) {
            if (frame->getName () == "WORLD") {
                EXPECT_FALSE (frameInGroup[0]);
                frameInGroup[0] = true;
            }
            else {
                const int id = std::stoi (frame->getName ().substr (5));
                EXPECT_FALSE (frameInGroup[id]);
                frameInGroup[id] = true;
            }
        }
    }
    for (std::size_t i = 0; i < frameInGroup.size (); i++) {
        EXPECT_TRUE (frameInGroup[i]);
    }
}
