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

#include "QPSolver.hpp"

#include <rw/core/macros.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <cmath>
#include <list>

using namespace rw::math;

using namespace rwlibs::algorithms::qpcontroller;

namespace {

const double EPS         = 1e-12;
const double ERROR_LIMIT = 1e-10;

typedef std::list< int > IntList;

const double QP_EPSILON = 1e-12;
}    // namespace

Eigen::VectorXd QPSolver::getInitialConfig (Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    size_t m = A.rows ();
    size_t n = A.cols ();

    Eigen::VectorXd x (n);

    for (size_t i = 0; i < n; i++) {
        x (i) = 0.5 * (b (2 * i) - b (2 * i + 1));
    }

    Eigen::VectorXd ax;
    ax = A * x;

    Eigen::VectorXd delta;

    if (ax (m - 1) + EPS < b (m - 1)) {
        for (int i = 0; i < b.size () - 1; i++) {
            Eigen::RowVectorXd grad = A.row (i);
            delta                   = A * grad;

            if (fabs (delta (m - 1)) > EPS) {
                double error = b (m - 1) - ax (m - 1);
                double dq1   = error / delta (m - 1);

                if (dq1 < 0) {
                    double limit = (b (i) - ax (i));
                    double dq2   = limit / (delta (i));
                    x += grad * std::max (dq1, dq2);
                    ax = A * x;
                }
            }
        }
    }
    return x;
}

Eigen::VectorXd QPSolver::safeApprox (Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    Eigen::VectorXd x = getInitialConfig (A, b);

    size_t m = A.rows ();

    Eigen::RowVectorXd grad = A.row (m - 1);
    double alpha            = (b (m - 1) - grad.dot (x) / grad.dot (grad));
    x += alpha * grad;
    return x;
}

/**
 * Solves the quadratic problem of minimizing 1/2 x^T.G.x+d^T.x subject to A.x>=b
 * The method used is an iterative method from "Numerical Optimization" by Jorge Nocedal
 * and Stephen J. Wright, Springer 1999.
 * In this implementation we'll require that b<=0 s.t. x=0 is a feasible initial value
 *
 * \param G The G matrix. It is required that G is n times n and is positive semidefinite
 * \param d Vector of length n
 * \param A Matrix used to represent the linear inequality constraints. The dimensions should be m
 * times n \param b Vector with the lower limit for the constraints. We'll assume that b<=0. The
 * length of b should be m
 */
Eigen::VectorXd QPSolver::inequalitySolve (const Eigen::MatrixXd& G, const Eigen::VectorXd& d,
                                           Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                           const Eigen::VectorXd& xstart, Status& status)
{
    const size_t n = G.rows ();
    const size_t m = A.rows ();

    //   bool finished = false;
    // size_t itcnt = 0;
    Eigen::RowVectorXd jz = A.row (m - 1);

    Eigen::VectorXd x = xstart;    // getInitialConfig(A, b);

    Eigen::VectorXd bcompare = A * x;
    for (int i = 0; i < b.size (); i++)
        if (bcompare (i) + ERROR_LIMIT < b (i)) {
            status = FAILURE;
            std::cout << "Warning: Invalid start configuration" << i << "   "
                      << bcompare (i) - b (i) << std::endl;
            //    return xstart;
        }

    IntList Wk;
    IntList notWk;

    // We initialize Wk to contain all the active constraints
    for (size_t i = 0; i < m; i++)
        if (fabs (b[i] - bcompare[i]) < EPS)
            Wk.push_back ((int) i);
        else
            notWk.push_back ((int) i);

    for (size_t i = 0; i < n * (m + 1); i++) {
        /*
         * Construct  |G   A_k^T | |p      |  |-g_k|
         *            |A_k 0     |.|\lambda|= | 0  |
         * where g_k=G.x_k+d
         */
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero (n + Wk.size (), n + Wk.size ());

        Eigen::VectorXd rhs = Eigen::VectorXd::Zero (n + Wk.size ());

        for (int i = 0; i < G.rows (); i++)
            for (int j = 0; j < G.rows (); j++)
                M (i, j) = G (i, j);
        size_t next = n;
        for (IntList::const_iterator ci = Wk.begin (); ci != Wk.end (); ci++) {
            for (size_t j = 0; j < n; j++) {
                M (next, j) = A (*ci, j);
                M (j, next) = -A (*ci, j);
            }
            ++next;
        }
        Eigen::VectorXd g_k;
        g_k = G * x + d;
        for (int i = 0; i < g_k.size (); i++)
            rhs (i) = -g_k (i);

        Eigen::MatrixXd Minv = LinearAlgebra::pseudoInverse (M, 1e-12);

        Eigen::VectorXd pl (Minv * rhs);

        Eigen::VectorXd p_k (n);
        for (size_t i = 0; i < n; i++)
            p_k (i) = pl (i);

        // Is the step zero
        //      std::cout<<"p_k = "<<p_k<<std::endl;
        if (p_k.lpNorm< Eigen::Infinity > () < 1e-8) {
            Eigen::VectorXd lambda;
            lambda = pl.tail (n - pl.size ());

            // std::cout<<"lambda = "<<min(lambda)<<std::endl;

            if (Math::min (lambda) >= -QP_EPSILON) {
                Eigen::VectorXd bcompare (x.size ());
                bcompare = A * x;
                for (int i = 0; i < b.size (); i++)

                    if (bcompare (i) + ERROR_LIMIT < b (i)) {
                        std::cout << "Warning: Could not find valid result for " << i
                                  << " error = " << (b (i) - bcompare (i)) << std::endl;

                        std::cout << "Returns standard solution" << std::endl;

                        status = SUBOPTIMAL;
                        return xstart;
                    }
                status = SUCCESS;
                return x;
            }
            else {
                IntList::iterator min = Wk.begin ();
                int next              = 0;
                double minVal         = lambda (next);
                for (IntList::iterator it = Wk.begin (); it != Wk.end (); ++it) {
                    if (lambda (next) < minVal) {
                        min    = it;
                        minVal = lambda (next);
                    }
                    ++next;
                }
                notWk.push_back (*min);
                Wk.erase (min);
            }
        }
        else {
            // compute \alpha_k as min(1,min (b_i-a_i^T x_k)/(a_i^T p_k) with i\notin W_k,
            // a_i^Tp_k<0

            double minval = 1;
            IntList::iterator blockingConstraint;
            bool blocked = false;
            for (IntList::iterator it = notWk.begin (); it != notWk.end (); it++) {
                double b_i = b[*it];
                Eigen::VectorXd a_i (A.cols ());
                for (int i = 0; i < A.cols (); i++)
                    a_i (i) = A (*it, i);
                double a_iTp_k = a_i.dot (p_k);

                if (a_iTp_k < 0) {
                    double val = (b_i - a_i.dot (x)) / (a_iTp_k);

                    if (val <= minval) {
                        minval             = val;
                        blockingConstraint = it;
                        blocked            = true;
                    }
                }
            }
            double alpha = std::min (1.0, minval);
            x            = x + (p_k * alpha);
            if (blocked) {
                Wk.push_back (*blockingConstraint);
                notWk.erase (blockingConstraint);
            }
        }
    }

    RW_WARN ("QPSolver did not terminate correctly. This may be due to round off error.");

    bcompare = A * x;

    for (int i = 0; i < b.size (); i++) {
        if (bcompare (i) + ERROR_LIMIT < b (i)) {
            std::cout << "QPSolver Failed to find result valid for all constraints " << i
                      << std::endl;
            RW_WARN ("QPSolver failed to find a result valid for Constraint " << i);
            status = FAILURE;
            return xstart;
        }
    }

    status = SUBOPTIMAL;

    return x;
}
