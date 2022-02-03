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

#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_QPSOLVER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_QPSOLVER_HPP

/**
 * @file QPSolver.hpp
 */

#include <Eigen/Core>

namespace rwlibs { namespace algorithms { namespace qpcontroller {

    /**
     * @brief Class providing an algorithms for solving the quadratic optimization
     * problem associated with the QPController
     */
    class QPSolver
    {
      public:
        /**
         * @brief Enumeration used to indicate status
         */
        enum Status {
            SUCCESS = 0, /* Solved */
            SUBOPTIMAL, /* Constraint satisfied but the result may be suboptimal. This may occur due
                           to round off errors */
            FAILURE     /* Could not find a solution statisfying all the constraints */
        };

        /**
         * Solves the quadratic problem of minimizing 1/2 x^T.G.x+d^T.x subject
         * to A.x>=b The method used is an iterative method from "Numerical
         * Optimization" by Jorge Nocedal and Stephen J. Wright, Springer 1999.
         * In this implementation we'll require that b<= s.t. x=0 is a feasible
         * initial value
         *
         * \param G [in] The G matrix. It is required that G is n times n and is
         * positive semidefinite
         *
         * \param d [in] Vector of length n
         *
         * \param A [in] Matrix used to represent the linear inequality constraints.
         * The dimensions should be m times n
         *
         * \param b [in] Vector with the lower limit for the constraints. We'll
         * assume that b<=0. The length of b should be m
         *
         * \param xstart [in] Default start configuration of the iterative algorithm
         *
         * \param status [out] Gives the status of the solving
         */
        static Eigen::VectorXd inequalitySolve (const Eigen::MatrixXd& G, const Eigen::VectorXd& d,
                                                Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                                const Eigen::VectorXd& xstart, Status& status);

        // TODO Investigate the possibility of making a hot-start of the
        // algorithm for better performance

      private:
        static Eigen::VectorXd getInitialConfig (Eigen::MatrixXd& A, const Eigen::VectorXd& b);

        static Eigen::VectorXd safeApprox (Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    };

}}}    // namespace rwlibs::algorithms::qpcontroller

#endif    // end include guard
