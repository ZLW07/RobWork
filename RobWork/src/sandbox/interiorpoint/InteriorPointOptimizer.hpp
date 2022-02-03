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

#ifndef RW_MATH_INTERIORPOINTOPTIMIZER_HPP
#define RW_MATH_INTERIORPOINTOPTIMIZER_HPP

#include <rw/math/Math.hpp>

#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace rw {
namespace math {


class InteriorPointOptimizer
{
public:

    typedef boost::function<void(const Eigen::VectorXd& x,
                                 double& f,
                                 Eigen::VectorXd& df,
                                 Eigen::MatrixXd& ddf) > ObjectFunction;

    typedef void*(Q& q) ObjectFunction;

    typedef boost::function<void(const Eigen::VectorXd& x,
                                 size_t no,
                                 Eigen::VectorXd& g,
                                 Eigen::MatrixXd& dg,
                                 Eigen::MatrixXd& ddq) > ConstraintFunction;

    InteriorPointOptimizer(size_t n,
                           size_t m,
                           ObjectFunction objectFunction,
                           ConstraintFunction constraintFunction);

    virtual ~InteriorPointOptimizer();


    int solve(const Eigen::VectorXd& x_init);

    void setAccuracy(double accuracy);
    double getAccuracy();

    void verify_user_defined_objective_and_constraints();


protected:
    InteriorPointOptimizer(size_t n, size_t m);

    void initialize();

    virtual void objectFunction(const Eigen::VectorXd& x,
                                double &f,
                                Eigen::VectorXd &df,
                                Eigen::MatrixXd &ddf);

    virtual void constraintFunction(const Eigen::VectorXd& x,
                                    int i,
                                    Eigen::VectorXd &a,
                                    Eigen::MatrixXd &da,
                                    Eigen::MatrixXd &dda);

private:
    ObjectFunction compute_f_info_EXT;
    ConstraintFunction compute_con_info_i_EXT;


    void choleskySolve(int n_e,
                       int bw,
                       Eigen::MatrixXd &A,
                       Eigen::VectorXd &b,
                       Eigen::VectorXd &x);


    void compute_f_info(Eigen::MatrixXd &A,
                        Eigen::VectorXd &RHS);

    void compute_con_info(Eigen::MatrixXd &A,
                          Eigen::VectorXd &RHS);

    void merit_info(Eigen::VectorXd &x,
                    Eigen::VectorXd &s,
                    double &phi,
                    double &eta);

    void Dmerit_info(Eigen::VectorXd &x,
                     Eigen::VectorXd &s,
                     Eigen::VectorXd &dx,
                     Eigen::VectorXd &ds,
                     double &Dphi,
                     double &eta);

    void update(Eigen::VectorXd &x,
                Eigen::VectorXd &dx,
                Eigen::VectorXd &s,
                Eigen::VectorXd &z);

    const size_t N;
    const size_t M;
    double _accuracy;
    Eigen::VectorXd _x;
    Eigen::VectorXd _s;
    Eigen::VectorXd _z;
    double _mu, _eta;

    // objective and derivatives
    double _f;
    Eigen::VectorXd _df;
    Eigen::MatrixXd _ddf;

    // constraints and derivatives (second derivative only stored for one constraint at a time)
    Eigen::VectorXd _a;
    Eigen::MatrixXd _da;
    Eigen::MatrixXd _dda;


};

} //end namespace math
} //end namespace rw

#endif //end include guard
