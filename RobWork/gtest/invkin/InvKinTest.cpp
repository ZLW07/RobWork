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

#include <gtest/gtest.h>

#include <rw/invkin.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/DHParameterSet.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/core/Ptr.hpp>

#include "../TestEnvironment.hpp"

#include <string>

using rw::core::ownedPtr;
using namespace rw::invkin;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

typedef IterativeIK::Ptr (* MakeIKSolver)(SerialDevice*, State&);
typedef IterativeMultiIK::Ptr (* MakeMultiIKSolver)(TreeDevice*, State&);

int testIKSolver(
    const std::string& solverName,
    MakeIKSolver maker,
    double relativeDisplacement)
{
    // Load a serial device that has revolute joints only.
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load(TestEnvironment::testfilesDir() + "PA10/pa10.xml");
    Device* any_device = workcell->getDevices().at(0).get();
    SerialDevice* device = dynamic_cast<SerialDevice*>(any_device);
    EXPECT_TRUE(device);

    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    const int maxCnt = 10;

    // The maximum amounts with which each joint is displaced.
    const Q displacements =
        relativeDisplacement * (pair.second - pair.first);

    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {

        Q q = q_zero;
        const int dof = (int)q_zero.size();
        for (int i = 0; i < dof; i++) {
            const double d = displacements(i);
            q(i) += Math::ran(-d, d);
        }

        q_targets.push_back(q);
    }

    // Use these to create targets.
    std::vector<Transform3D<> > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        targets.push_back(device->baseTend(state));
    }

    State initial_state = workcell->getDefaultState();
    IterativeIK::Ptr solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.

    device->setQ(q_zero, initial_state);
    unsigned int errcnt = 0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if (!ok) {
            std::cout << "Could not solve IK for solver " << solverName << "\n";
            errcnt++;
        }
    }

    return errcnt; 
}

int testMultiIKSolver( 
    const std::string& solverName,
    MakeMultiIKSolver maker,  
    double relativeDisplacement) 
{
    // Load a tree device that has revolute joints only.
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load(
        TestEnvironment::testfilesDir() + "SchunkHand/SchunkHand.xml");  

    Device* any_device = workcell->getDevices().at(0).get();

    TreeDevice* device = dynamic_cast<TreeDevice*>(any_device);
    EXPECT_TRUE(device);
    //std::cout << "Device loadet" << std::endl;
    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds(); 
    const Q q_zero = 0.45 * (pair.first + pair.second);

    const int maxCnt = 10;

    // The maximum amounts with which each joint is displaced.
    const Q displacements =
        relativeDisplacement * (pair.second - pair.first);
    //std::cout << "Calculate random configurations" << std::endl;

    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {

        Q q = q_zero;
        const int dof = (int)q_zero.size();
        for (int i = 0; i < dof; i++) {
            const double d = displacements(i);
            q(i) += Math::ran(-d, d);
        }

        q_targets.push_back(q);
    }
    //std::cout << "Calculate random targets" << std::endl;
    // Use these to create targets.
    std::vector<std::vector<Transform3D<> > > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        std::vector<Transform3D<> > target;
        for(size_t j=0;j<device->getEnds().size();j++){
            target.push_back(device->baseTframe(device->getEnds()[j],state));
        }
        targets.push_back(target);
    }
    //std::cout << "get intial state\n";
    State initial_state = workcell->getDefaultState();
    IterativeMultiIK::Ptr solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.
    //std::cout << "Solve IK: " << solverName << "\n";
    device->setQ(q_zero, initial_state);
    unsigned int errcnt = 0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if (!ok) {
            //std::cout << "Could not solve IK for solver " << solverName << "\n";
            errcnt++;
        }
    }

    return errcnt;
}


IterativeIK::Ptr makeCCD(SerialDevice* device, State& state)
{
    return ownedPtr(new CCDSolver(device,state));
}

IterativeIK::Ptr makeJacobianIKSolverSVD(SerialDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolver> sol = ownedPtr(new JacobianIKSolver(device,state));
    // Should use SVD as default though
    sol->setSolverType(JacobianIKSolver::SVD);
    IterativeIK::Ptr result(sol);
    return result;
}

IterativeIK::Ptr makeJacobianIKSolverTranspose(SerialDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolver> sol = ownedPtr(new JacobianIKSolver(device,state));
    sol->setSolverType(JacobianIKSolver::Transpose);
    sol->setMaxIterations(16);
    IterativeIK::Ptr result(sol);
    return result;
}

IterativeIK::Ptr makeJacobianIKSolverDLS(SerialDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolver> sol = ownedPtr(new JacobianIKSolver(device,state));
    sol->setSolverType(JacobianIKSolver::DLS);
    sol->setMaxIterations(50);
    IterativeIK::Ptr result(sol);
    return result;
}
/*
IterativeIK::Ptr makeIKQPSolver(SerialDevice* device, State& state) {
  IterativeIK::Ptr result(ownedPtr(new IKQPSolver(device, state)));
  return result;
}
*/

IterativeMultiIK::Ptr makeJacobianIKSolverMSVD(TreeDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolverM> sol = ownedPtr(new JacobianIKSolverM(device, state));
    // Should use SVD as default though
    sol->setSolverType(JacobianIKSolverM::SVD);
    //sol->setMaxLocalStep(0.4,5.0);
    IterativeMultiIK::Ptr result(sol);
    return result;
}

IterativeMultiIK::Ptr makeJacobianIKSolverMTranspose(TreeDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolverM> sol = ownedPtr(new JacobianIKSolverM(device, state));
    // Should use SVD as default though
    sol->setSolverType(JacobianIKSolverM::Transpose);
    sol->setMaxIterations(350);
    IterativeMultiIK::Ptr result(sol);
    return result;
}

IterativeMultiIK::Ptr makeJacobianIKSolverMDLS(TreeDevice* device, State& state)
{
    rw::core::Ptr<JacobianIKSolverM> sol = ownedPtr(new JacobianIKSolverM(device, state));
    // Should use SVD as default though
    sol->setSolverType(JacobianIKSolverM::DLS);
    sol->setMaxIterations(155);
    IterativeMultiIK::Ptr result(sol);
    return result;
}

TEST(InvKinTest, testIterativeInverseKinematics )
{
    int errcnt = 0;
    // We seed the random number generator so that we get reproducible results.
    Math::seed(0);

    // The IK solvers really aren't impressive right now. In particular the CCD
    // solver isn't very reliable. Some tuning of these is necessary, I think.
    // Also perhaps the testIKSolver() should just verify that a _reasonably_
    // large percentage of the IK calculations succeed.

    // The relative displacement has been set to a small enough number to ensure that only a few iterations are needed to find a solution for the Transpose and DLS solvers
    // Increase the number of allowed iterations accordingly to the chosen relative displacement
 

    // Too slow to be considered correct.
    errcnt = testIKSolver("CCD", makeCCD, 0.002);
    EXPECT_EQ(errcnt, 0);
    //testIKSolver("IKQPSolver", makeIKQPSolver, 0.2);
    errcnt = testIKSolver("JacobianIKSolver using SVD", makeJacobianIKSolverSVD, 0.2);
    EXPECT_EQ(errcnt, 0);
    //errcnt = testIKSolver("JacobianIKSolver using Transpose", makeJacobianIKSolverTranspose, 0.00002);
    EXPECT_LE(errcnt, 2); // maxIteration is set low enough to not find a solution for up to 2 of the tests
    //errcnt = testIKSolver("JacobianIKSolver using DLS", makeJacobianIKSolverDLS, 0.00002);
    EXPECT_EQ(errcnt, 0);

    //errcnt = testMultiIKSolver("JacobianIKSolverM using SVD", makeJacobianIKSolverMSVD, 0.2);
    //BOOST_CHECK_EQUAL(errcnt, 0);
    //errcnt = testMultiIKSolver("JacobianIKSolverM using Transpose", makeJacobianIKSolverMTranspose, 0.00002);
    //BOOST_CHECK_EQUAL(errcnt, 0);
    //errcnt = testMultiIKSolver("JacobianIKSolverM using DLS", makeJacobianIKSolverMDLS, 0.00002);
    //BOOST_CHECK_LE(errcnt, 2); // maxIteration is set low enough to not find a solution for up to 2 of the tests
}

int testClosedFormWithQ(const Q& q, std::vector<DHParameterSet>& dhparams) {
    //Transform from the three intersection axis to tool
    Transform3D<> T06(Transform3D<>::identity());

    for (size_t i = 0; i<dhparams.size(); i++) {
        T06 = T06*Transform3D<>::craigDH(
            dhparams[i].alpha(),
            dhparams[i].a(),
            dhparams[i].d(),
            q(i));
    }
    Transform3D<> T6tool(Vector3D<>(0.1,0.2,0.3), RPY<>(1,2,3));

    Transform3D<> baseTend = T06*T6tool;

    PieperSolver solver(dhparams, T6tool);
    State state;
    std::vector<Q> solutions = solver.solve(baseTend, state);

    //    BOOST_CHECK(solutions.size() == 8);
    for (std::vector<Q>::iterator it = solutions.begin(); it != solutions.end(); ++it) {
        Q qres = *it;
        T06 = Transform3D<>::identity();
        for (size_t i = 0; i<dhparams.size(); i++) {
            T06 = T06*Transform3D<>::craigDH(
                dhparams[i].alpha(),
                dhparams[i].a(),
                dhparams[i].d(),
                qres(i));
        }

        Transform3D<> T6tool(Vector3D<>(0.1,0.2,0.3), RPY<>(1,2,3));
        Transform3D<> baseTend2 = T06*T6tool;

        Transform3D<> diff = inverse(baseTend)*baseTend2;

        for (int i = 0; i<3; i++) {
            for (int j = 0; j<4; j++) {
                if (i == j)
                    EXPECT_LT(fabs(diff(i,j)-1) , 1e-12);
                else
                    EXPECT_LT(fabs(diff(i,j)) , 1e-12);
            }
        }
    }
    return (int)solutions.size();
}


TEST(InvKinTest, testClosedFormInverseKinematics ) {
    //std::cout<<"- Testing PieperSolver"<<std::endl;
    Q q(Q::Base::Zero(6));

    std::vector<DHParameterSet> dhparams;
    dhparams.push_back(DHParameterSet(0,0,0,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad, 0.26, 0, 0));
    dhparams.push_back(DHParameterSet(0,0.68,0,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams.push_back(DHParameterSet(90*Deg2Rad,0,0,0));


    int cnt = testClosedFormWithQ(q, dhparams);
    EXPECT_EQ(cnt , 8);

    q(0) = 0.5;
    q(1) = 1;
    q(2) = 1.5;
    q(3) = 2;
    q(4) = 2.5;
    q(5) = 3;
    cnt = testClosedFormWithQ(q, dhparams);
    EXPECT_EQ(cnt , 8);


    //Test special case with a1=0
    std::vector<DHParameterSet> dhparams2;
    dhparams2.push_back(DHParameterSet(0,0,0,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad, 0, 0, 0));
    dhparams2.push_back(DHParameterSet(0,0.68,0,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams2.push_back(DHParameterSet(90*Deg2Rad,0,0,0));
    cnt = testClosedFormWithQ(q, dhparams2);
    EXPECT_EQ(cnt , 8);

    //Test special case with alpha1 = 0
    std::vector<DHParameterSet> dhparams3;


    dhparams3.push_back(DHParameterSet(0,0,0,0));
    dhparams3.push_back(DHParameterSet(0, 0.26, 0, 0));
    dhparams3.push_back(DHParameterSet(90*Deg2Rad,0.68,0,0));
    dhparams3.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams3.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams3.push_back(DHParameterSet(90*Deg2Rad,0,0,0));

    q(0) = 1;
    q(1) = 1;
    q(2) = 1;
    q(3) = 1.4;
    q(4) = 1.5;
    q(5) = 1.6;

    cnt = testClosedFormWithQ(q, dhparams3);
    EXPECT_EQ(cnt , 4);
    // std::cout<<"PieperSolver Tested"<<std::endl;
}
