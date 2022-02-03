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

#include "LinearAlgebra.hpp"

#include <rw/core/macros.hpp>

#include <Eigen/SVD>

using namespace rw::math;

Eigen::MatrixXd LinearAlgebra::pseudoInverse (const Eigen::MatrixXd& am, double precision)
{
    if (am.rows () < am.cols ()) {
        // RW_THROW("pseudoInverse require rows >= to cols!");
        Eigen::MatrixXd a = am.transpose ();
        Eigen::JacobiSVD< Eigen::MatrixXd > svd =
            a.jacobiSvd (Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = precision * std::max (a.cols (), a.rows ()) *
                           svd.singularValues ().array ().abs ().maxCoeff ();
        return (svd.matrixV () *
                Eigen::MatrixXd ((svd.singularValues ().array ().abs () > tolerance)
                                     .select (svd.singularValues ().array ().inverse (), 0))
                    .asDiagonal () *
                svd.matrixU ().adjoint ())
            .transpose ();
    }
    else {
        Eigen::JacobiSVD< Eigen::MatrixXd > svd =
            am.jacobiSvd (Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = precision * std::max (am.cols (), am.rows ()) *
                           svd.singularValues ().array ().abs ().maxCoeff ();
        return svd.matrixV () *
               Eigen::MatrixXd ((svd.singularValues ().array ().abs () > tolerance)
                                    .select (svd.singularValues ().array ().inverse (), 0))
                   .asDiagonal () *
               svd.matrixU ().adjoint ();
    }
}

void LinearAlgebra::svd (const Eigen::MatrixXd& M, Eigen::MatrixXd& U, Eigen::VectorXd& sigma,
                         Eigen::MatrixXd& V)
{
    const Eigen::JacobiSVD< Eigen::MatrixXd > svd =
        M.jacobiSvd (Eigen::ComputeFullU | Eigen::ComputeFullV);
    U     = svd.matrixU ();
    sigma = svd.singularValues ();
    V     = svd.matrixV ();
}

bool LinearAlgebra::checkPenroseConditions (const Eigen::MatrixXd& A, const Eigen::MatrixXd& X,
                                            double prec)
{
    const Eigen::MatrixXd AX = A * X;
    const Eigen::MatrixXd XA = X * A;

    if (((AX * A) - A).lpNorm< Eigen::Infinity > () > prec)
        return false;
    if (((XA * X) - X).lpNorm< Eigen::Infinity > () > prec)
        return false;
    if ((AX.transpose () - AX).lpNorm< Eigen::Infinity > () > prec)
        return false;
    if ((XA.transpose () - XA).lpNorm< Eigen::Infinity > () > prec)
        return false;
    return true;
}

template<>
std::pair< LinearAlgebra::EigenMatrix< double >::type, LinearAlgebra::EigenVector< double >::type >
LinearAlgebra::eigenDecompositionSymmetric< double > (const Eigen::MatrixXd& Am1)
{
    Eigen::SelfAdjointEigenSolver< Eigen::MatrixXd > eigenSolver;
    eigenSolver.compute (Am1);
    return std::make_pair (eigenSolver.eigenvectors (), eigenSolver.eigenvalues ());
}

template<>
std::pair< LinearAlgebra::EigenMatrix< std::complex< double > >::type,
           LinearAlgebra::EigenVector< std::complex< double > >::type >
LinearAlgebra::eigenDecomposition< double > (const Eigen::MatrixXd& Am1)
{
    Eigen::EigenSolver< Eigen::MatrixXd > eigenSolver;
    eigenSolver.compute (Am1);
    return std::make_pair (eigenSolver.eigenvectors (), eigenSolver.eigenvalues ());
}
