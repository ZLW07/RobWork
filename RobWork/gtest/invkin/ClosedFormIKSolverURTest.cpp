/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>

#include "../TestEnvironment.hpp"

using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;
using rw::invkin::ClosedFormIKSolverUR;

TEST(ClosedFormIKSolverURTest, ClosedFormIKSolverURTest ){
	static const double EPS = 1e-14;


    {
        const WorkCell::Ptr wc = WorkCellLoader::Factory::load(TestEnvironment::testfilesDir() + "devices/UR6855A/UR6855A.wc.xml");
        EXPECT_FALSE(wc.isNull());
        SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR-6-85-5-A");
        EXPECT_FALSE(device.isNull());
        const ClosedFormIKSolverUR solver(device,wc->getDefaultState());

        State state = wc->getDefaultState();
        const Q qRef = Q(6,0.352,-2.408,-0.785,-1.78,2.199,0.785);
        device->setQ(qRef,state);
        const Transform3D<> T = device->baseTend(state);
        const std::vector<Q> solutions = solver.solve(T, state);
        EXPECT_GT(solutions.size() , 0);
        bool found = false;
        for(const Q& sol : solutions) {
            if ((sol-qRef).normInf() <= EPS) {
                found = true;
            }
            device->setQ(sol,state);
            const Transform3D<> Tfound = device->baseTend(state);
            EXPECT_LT((Tfound.P()-T.P()).normInf(),EPS);
            EXPECT_TRUE(T.R().equal(Tfound.R(),EPS));
        }
        EXPECT_TRUE(found);
    }

    {
        const WorkCell::Ptr wc = WorkCellLoader::Factory::load(TestEnvironment::testfilesDir() + "devices/UR5e/UR5e.xml");
        EXPECT_FALSE(wc.isNull());
        SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR5e_2018");
        EXPECT_FALSE(device.isNull());
        const ClosedFormIKSolverUR solver(device,wc->getDefaultState());

        State state;
        Transform3D<> T;
        std::vector<Q> solutions;

        state = wc->getDefaultState();
        const Q qRef = Q(6,0.352,-2.408,-0.785,-1.78,2.199,0.785);
        device->setQ(qRef,state);
        T = device->baseTend(state);
        solutions = solver.solve(T, state);
        EXPECT_GT(solutions.size() , 0);
        bool found = false;
        for(const Q& sol : solutions) {
            if ((sol-qRef).normInf() <= EPS) {
                found = true;
            }
            device->setQ(sol,state);
            const Transform3D<> Tfound = device->baseTend(state);
            EXPECT_LT((Tfound.P()-T.P()).normInf(),EPS);
            EXPECT_TRUE(T.R().equal(Tfound.R(),EPS));
        }
        EXPECT_TRUE(found);

        state = wc->getDefaultState();
        T = Transform3D<>(Vector3D<>(0.0853719, -0.0565455, 0.725042), RPY<>(-0.785398, 3.16998e-17, 4.71377e-17));
        solutions = solver.solve(T, state);
        EXPECT_EQ(solutions.size() , 0);
    }
}
