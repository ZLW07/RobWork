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

#ifndef RW_MATH_LINEARALGEBRA_HPP
#define RW_MATH_LINEARALGEBRA_HPP

/**
 * @file LinearAlgebra.hpp
 */
#if !defined(SWIG)
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <limits>
#endif 
namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief Collection of Linear Algebra functions
     */
    class LinearAlgebra
    {
      public:
        //! @brief Type for Eigen matrices used to reduce namespace cluttering.
        template< class T = double > struct EigenMatrix
        {
            //! type of this matrix
            typedef Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > type;
        };

        //! @brief Type for Eigen vectors, used to reduce namespace cluttering.
        template< class T = double > struct EigenVector
        {
            //! type of this Vector
            typedef Eigen::Matrix< T, Eigen::Dynamic, 1 > type;
        };

        /**
         * @brief Performs a singular value decomposition (SVD)
         *
         * The SVD computes the decomposition
         * \f$ \mathbf{M}=\mathbf{U}*\mathbf{DiagonalMatrix(\sigma)}*\mathbf{V}^T \f$ .
         *
         * @param M [in] the matrix to decomposite
         * @param U [out] Result matrix \f$\mathbf{U}\f$
         * @param sigma [out] The \f$\mathbf{sigma}\f$ vector with diagonal elements
         * @param V [out] Result matrix \f$\mathbf{V}\f$
         */
        static void svd (const Eigen::Matrix<double,-1,-1>& M, Eigen::Matrix<double,-1,-1>& U, Eigen::Matrix<double,-1,1>& sigma,
                         Eigen::Matrix<double,-1,-1>& V);

        /**
         * \brief Calculates the moore-penrose (pseudo) inverse of a matrix
         * @f$ \mathbf{M}^+@f$
         *
         * \param am [in] the matrix @f$ \mathbf{M} @f$ to be inverted
         *
         * \param precision [in] the precision to use, values below this
         * treshold are considered singular
         *
         * \return the pseudo-inverse @f$ \mathbf{M}^+@f$ of @f$ \mathbf{M} @f$
         *
         * \f$ \mathbf{M}^+=\mathbf{V}\mathbf{\Sigma} ^+\mathbf{U}^T \f$ where
         * \f$ \mathbf{V} \f$, \f$ \mathbf{\Sigma} \f$ and \f$ \mathbf{U} \f$
         * are optained using Singular Value Decomposition (SVD)
         *
         *
         */
        static Eigen::Matrix<double,-1,-1> pseudoInverse (const Eigen::Matrix<double,-1,-1>& am, double precision = 1e-6);

        /**
         * @brief Checks the penrose conditions
         * @param A [in] a matrix
         * @param X [in] a pseudoinverse of A
         * @param prec [in] the tolerance
         *
         * @return true if the pseudoinverse X of A fullfills the penrose
         * conditions, false otherwise
         *
         * Checks the penrose conditions:
         *
         * @f$
         * AXA = A
         * @f$
         *
         * @f$
         * XAX = X
         * @f$
         *
         * @f$
         * (AX)^T = AX
         * @f$
         *
         * @f$
         * (XA)^T = XA
         * @f$
         */
        static bool checkPenroseConditions (const Eigen::Matrix<double,-1,-1>& A, const Eigen::Matrix<double,-1,-1>& X,
                                            double prec);

        /**
         * \brief Calculates matrix determinant
         * \param m [in] a square matrix
         * \return the matrix determinant
         */
        template< class R > static inline double det (const Eigen::MatrixBase< R >& m)
        {
            return m.determinant ();
        }

        /**
         * @brief Calculates matrix inverse.
         * @param M [in] input matrix @f$ \mathbf{M} @f$ to invert
         * @return output matrix @f$ \mathbf{M}^{-1} @f$
         **/
        template< class T > static T inverse (const Eigen::MatrixBase< T >& M)
        {
            return M.inverse ();
        }

        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template< class R > static inline bool isSO (const Eigen::MatrixBase< R >& M)
        {
            return M.cols () == M.rows () && isProperOrthonormal (M);
        }

        /**
         * @brief Checks if a given matrix is in SO(n) (special orthogonal)
         * @param M [in] \f$ \mathbf{M} \f$
         * @param precision [in] the precision to use for floating point comparison
         * @return true if \f$ M\in SO(n) \f$
         *
         * \f$ SO(n) = {\mathbf{R}\in \mathbb{R}^{n\times n} :
         * \mathbf{R}\mathbf{R}^T=\mathbf{I}, det \mathbf{R}=+1} \f$
         *
         */
        template< class R >
        static inline bool isSO (const Eigen::MatrixBase< R >& M, typename R::Scalar precision)
        {
            return M.cols () == M.rows () && isProperOrthonormal (M, precision);
        }

        /**
         * @brief Checks if a given matrix is skew-symmetrical
         * @param M [in] \f$ \mathbf{M} \f$ the matrix to check
         *
         * @return true if the property
         * \f$ \mathbf{M}=-\mathbf{M}^T \f$ holds,
         * false otherwise.
         */
        template< class R > static inline bool isSkewSymmetric (const Eigen::MatrixBase< R >& M)
        {
            return (M + M.transpose ()).template lpNorm< Eigen::Infinity > () == 0.0;
        }

        /**
         * @brief Checks if a given matrix is proper orthonormal
         * @return true if the matrix is proper orthonormal, false otherwise
         *
         * A matrix is proper orthonormal if it is orthonormal and its determinant
         * is equal to \f$ +1 \f$
         */
        template< class R >
        static inline bool isProperOrthonormal (
            const Eigen::MatrixBase< R >& r,
            typename R::Scalar precision = std::numeric_limits< typename R::Scalar >::epsilon ())
        {
            return isOrthonormal (r, precision) && fabs (r.determinant () - 1.0) <= precision;
        }

        /**
         * @brief Checks if a given matrix is orthonormal
         * @return true if the matrix is orthonormal, false otherwise
         *
         * A matrix is orthonormal if all of it's column's are mutually orthogonal
         * and all of it's column's has unit length.
         *
         * that is for any \f$ i, j \f$ the following holds
         * \f$ col_i . col_j = 0 \f$ and \f$ ||col_i|| = 1 \f$
         *
         * Another nessesary and sufficient condition of orthonormal matrices is that
         * \f$ \mathbf{M}\mathbf{M}^T=I \f$
         */
        template< class R >
        static inline bool isOrthonormal (
            const Eigen::MatrixBase< R >& r,
            typename R::Scalar precision = std::numeric_limits< typename R::Scalar >::epsilon ())
        {
            return (r * r.transpose ()).isIdentity (precision);
            // const Eigen::MatrixBase<R> m = r*r.transpose() ;
            // return m.isIdentity(1e-15);
            // double scale = m.norm();//m.lpNorm<Eigen::Infinity>();
            // return scale == 0.0;
        }

        /**
         * @brief Decomposition for a symmetric matrix.
         * @param Am1 [in] a symmetric matrix.
         * @return the decomposition as a pair with eigenvectors and eigenvalues.
         */
        template< class T >
        // static std::pair<typename EigenMatrix<T>::type, typename EigenVector<T>::type >
        // eigenDecompositionSymmetric(const typename EigenMatrix<T>::type& Am1)
        static std::pair< typename EigenMatrix< T >::type, typename EigenVector< T >::type >
        eigenDecompositionSymmetric (const Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic >& Am1)
        {
            Eigen::SelfAdjointEigenSolver< Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > >
                eigenSolver;
            eigenSolver.compute (Am1);
            return std::make_pair (eigenSolver.eigenvectors (), eigenSolver.eigenvalues ());
        }

        /**
         * @brief Eigen decomposition of a matrix.
         * @param Am1 [in] the matrix.
         * @return the decomposition as a pair with eigenvectors and eigenvalues.
         */
        template< class T >
        static std::pair< typename EigenMatrix< std::complex< T > >::type,
                          typename EigenVector< std::complex< T > >::type >
        eigenDecomposition (const typename Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic >& Am1)
        {
            Eigen::EigenSolver< Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > > eigenSolver;
            eigenSolver.compute (Am1);

            Eigen::Matrix< std::complex< T >, Eigen::Dynamic, Eigen::Dynamic > vectors =
                eigenSolver.eigenvectors ();
            Eigen::Matrix< std::complex< T >, Eigen::Dynamic, 1 > values =
                eigenSolver.eigenvalues ();
            return std::make_pair (vectors, values);
        }

      private:
    };

    template<>
    std::pair< typename LinearAlgebra::EigenMatrix< double >::type,
               typename LinearAlgebra::EigenVector< double >::type >
    LinearAlgebra::eigenDecompositionSymmetric< double > (const Eigen::MatrixXd& Am1);

    template<>
    std::pair< typename LinearAlgebra::EigenMatrix< std::complex< double > >::type,
               typename LinearAlgebra::EigenVector< std::complex< double > >::type >
    LinearAlgebra::eigenDecomposition< double > (const Eigen::MatrixXd& Am1);

    /*@}*/
}}    // namespace rw::math

#endif    // end include guard
