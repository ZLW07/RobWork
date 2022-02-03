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

#include "NullSpaceProjection.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/algorithms/qpcontroller/QPSolver.hpp>

using namespace rwlibs::algorithms;

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwlibs::algorithms::qpcontroller;

NullSpaceProjection::NullSpaceProjection (Device* device, Frame* controlFrame, const State& state,
                                          double dt) :
    _state (state),
    _dt (dt), _P (Eigen::MatrixXd::Identity (6, 6)), _space (BaseFrame)
{
    _device       = device;
    _controlFrame = controlFrame;
    _dof          = (int) _device->getDOF ();

    _qlower = _device->getBounds ().first;
    _qupper = _device->getBounds ().second;

    _dqlimit = _device->getVelocityLimits ();

    _ddqlimit = _device->getAccelerationLimits ();

    setThreshold (0.20);    // Only do self motion if joint is within the outmost 20% of its value

    _weightJointLimits = 1;
}

NullSpaceProjection::~NullSpaceProjection ()
{}

Q NullSpaceProjection::getGradient (const Q& q)
{
    Q g (_dof);
    for (int i = 0; i < _dof; i++) {
        if (q (i) > _thresholdUpper (i))
            g (i) =
                _weightJointLimits * (q (i) - _thresholdUpper (i)) / (_qupper (i) - _qlower (i));
        else if (q (i) < _thresholdLower (i))
            g (i) =
                _weightJointLimits * (q (i) - _thresholdLower (i)) / (_qupper (i) - _qlower (i));
        else
            g (i) = 0;
    }
    return -g;
}

Q NullSpaceProjection::solve (const Q& q, const Q& dqcurrent, const Q& dq1)
{
    Q lower (_dof);
    Q upper (_dof);

    calculateVelocityLimits (lower, upper, q, dqcurrent);
    lower -= dq1;
    upper -= dq1;

    _device->setQ (q, _state);

    // Calculate the right projection depending on whether it is in the base frame or the frame of
    // control
    Eigen::MatrixXd P;
    if (_space == ControlFrame) {
        Rotation3D<> rot     = inverse (_device->baseTframe (_controlFrame, _state).R ());
        Eigen::MatrixXd R    = Eigen::MatrixXd::Zero (6, 6);
        R.block (0, 0, 3, 3) = rot.e ();
        R.block (3, 3, 6, 6) = rot.e ();
        P                    = _P * R;
    }
    else {
        P = _P;
    }
    // Get the Jacobian and make the projection
    Eigen::MatrixXd jac = P * _device->baseJframe (_controlFrame, _state).e ();
    // Eigen::MatrixXd jac = _device->baseJframe(_controlFrame, _state).m();

    Eigen::MatrixXd jac_inv = LinearAlgebra::pseudoInverse (jac);

    Eigen::MatrixXd jac_ort = Eigen::MatrixXd::Identity (_dof, _dof) - jac_inv * jac;

    Eigen::MatrixXd cmat (2 * jac_ort.rows (), jac_ort.cols ());

    cmat.block (0, 0, jac_ort.rows (), jac_ort.cols ())                   = jac_ort;
    cmat.block (jac_ort.rows (), 0, 2 * jac_ort.rows (), jac_ort.cols ()) = -jac_ort;

    Eigen::VectorXd cvec (2 * jac_ort.rows ());
    cvec.head (lower.size ())                       = lower.e ();
    cvec.segment (lower.size (), 2 * lower.size ()) = -upper.e ();

    Eigen::MatrixXd jTj = jac_ort.transpose () * jac_ort;
    Eigen::VectorXd jTx = jac_ort.transpose () * getGradient (q).e ();

    Eigen::VectorXd qstart = Eigen::VectorXd::Zero (_dof);

    // std::cout<<"cvec = "<<cvec<<std::endl;

    QPSolver::Status status;
    Eigen::VectorXd res = QPSolver::inequalitySolve (jTj, -1 * jTx, cmat, cvec, qstart, status);
    if (status == QPSolver::SUBOPTIMAL)
        std::cout << "Returns something suboptimal" << std::endl;
    if (status == QPSolver::FAILURE)
        std::cout << "Returns Error" << std::endl;

    Eigen::VectorXd nsp = jac_ort * res;

    return Q (nsp);
}

void NullSpaceProjection::calculateVelocityLimits (Q& lower, Q& upper, const Q& q, const Q& dq)
{
    Q joint_pos = q;
    Q joint_vel = dq;

    double accmin, accmax, velmin, velmax, posmin, posmax;
    double x;

    for (int i = 0; i < _dof; i++) {
        // For the acceleration
        accmin = _dt * (-_ddqlimit)[i] + joint_vel (i);
        accmax = _dt * _ddqlimit[i] + joint_vel (i);
        // For the velocity
        velmin = -_dqlimit[i];
        velmax = _dqlimit[i];
        // For the position
        // If v_current<=v_max(X)
        x = _qupper[i] - joint_pos (i);
        if (x <= 0) {
            posmax = 0;
            //  std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
        }
        else {
            // For qmax
            double j_x = Math::round (sqrt (1 - 8 * x / (_dt * _dt * (-_ddqlimit)[i])) / 2 - 1);
            double q_end_x =
                (x + _dt * _dt * (-_ddqlimit)[i] * (j_x * (j_x + 1)) / 2) / (_dt * (j_x + 1));
            double q_max_x = q_end_x - j_x * (-_ddqlimit)[i] * _dt;
            double X       = x - _dt * q_max_x;
            if (X <= 0) {
                posmax = 0;
                //          std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
            }
            else {
                double j_X =
                    Math::round (sqrt (1. - 8 * X / (_dt * _dt * (-_ddqlimit)[i])) / 2. - 1);
                double q_end_X =
                    (X + _dt * _dt * (-_ddqlimit)[i] * (j_X * (j_X + 1)) / 2) / (_dt * (j_X + 1));
                posmax = q_end_X - j_X * (-_ddqlimit)[i] * _dt;
            }
        }
        x = joint_pos (i) - _qlower[i];
        if (x <= 0) {
            //  std::cout<<"Warning: Set lower pos limit to 0 because"<<x<<"<=0"<<std::endl;
            posmin = 0;
        }
        else {    // For qmin
            double j_x = Math::round (sqrt (1 + 8 * x / (_dt * _dt * _ddqlimit[i])) / 2 - 1);
            double q_end_x =
                (-x + _dt * _dt * _ddqlimit[i] * (j_x * (j_x + 1)) / 2) / (_dt * (j_x + 1));
            double q_min_x = q_end_x - j_x * _ddqlimit[i] * _dt;
            double X       = x + _dt * q_min_x;
            if (X <= 0) {
                posmin = 0;
                //   std::cout<<"Warning: Set lower pos limit to 0"<<x<<"<=0"<<std::endl;
            }
            else {
                double j_X = Math::round (sqrt (1 + 8 * X / (_dt * _dt * _ddqlimit[i])) / 2 - 1);
                double q_end_X =
                    (-X + _dt * _dt * _ddqlimit[i] * (j_X * (j_X + 1)) / 2) / (_dt * (j_X + 1));
                posmin = q_end_X - j_X * _ddqlimit[i] * _dt;
            }
        }
        upper (i) = std::min (accmax, std::min (velmax, posmax));
        lower (i) = std::max (accmin, std::max (velmin, posmin));

        // Because of numerical uncertainties we need to test whether upper>lower.
        if (upper (i) < lower (i)) {
            lower (i) = upper (i);
        }
    }
}

void NullSpaceProjection::setProjection (const Eigen::MatrixXd& P, ProjectionFrame space)
{
    _P     = P;
    _space = space;
}

void NullSpaceProjection::setThreshold (double threshold)
{
    _thresholdLower = _qlower + threshold * (_qupper - _qlower);
    _thresholdUpper = _qupper - threshold * (_qupper - _qlower);
}

void NullSpaceProjection::setJointLimitsWeight (double w)
{
    _weightJointLimits = w;
}
