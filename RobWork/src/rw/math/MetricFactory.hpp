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

#ifndef RW_MATH_METRICFACTORY_HPP
#define RW_MATH_METRICFACTORY_HPP

/**
   @file MetricFactory.hpp
*/
#if !defined(SWIG)
#include <rw/kinematics/State.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Metric.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Vector2D.hpp>
#endif

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    /**
       @brief Manhattan distance metric for vector types.

       The ManhattanMetric, also known as the taxicab metric or the 1-norm, is a
       metric on the Euclidean n-Plane. The Manhattan distance between two
       points

       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sum_{i=1}^{n} |p_i - q_i| \f$

       @relates Metric
    */
    template< class T > class ManhattanMetric : public Metric< T >
    {
      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::norm1 (q);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::dist1 (a, b);
        }
    };

    /**
       @brief Weighted Manhattan distance metric for vector types.

       Given a vector of weights \f$ \mathbf{\omega}\in\mathbb{R}^n \f$,
       the distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sum_{i=1}^{n} |\omega_i * (p_i - q_i)| \f$.

       @relates Metric
    */
    template< class T > class WeightedManhattanMetric : public Metric< T >
    {
      public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedManhattanMetric (const typename Metric< T >::value_type& weights) :
            _weights (weights)
        {}

      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::norm1Weighted (q, _weights);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::dist1Weighted (a, b, _weights);
        }

        int doSize () const { return (int) _weights.size (); }

        const typename Metric< T >::value_type _weights;
    };

    /**
       @brief Euclidean distance metric for vector types.

       The distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sqrt{\sum_{i=1}^{n}(p_i - q_i)^2} \f$

       @relates Metric
    */
    template< class T > class EuclideanMetric : public Metric< T >
    {
      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::norm2 (q);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::dist2 (a, b);
        }
    };

    /**
       @brief Weighted Euclidean metric for vector types.

       Given a vector of weights \f$ \mathbf{\omega}\in\mathbb{R}^n \f$,
       the distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ \sqrt{\sum_{i=1}^{n}(\omega_i * (p_i - q_i))^2} \f$.

       @relates Metric
    */
    template< class T > class WeightedEuclideanMetric : public Metric< T >
    {
      public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedEuclideanMetric (const typename Metric< T >::value_type& weights) :
            _weights (weights)
        {}

      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::norm2Weighted (q, _weights);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::dist2Weighted (a, b, _weights);
        }

        int doSize () const { return (int) _weights.size (); }

        const typename Metric< T >::value_type _weights;
    };

    /**
       @brief Infinity norm distance metric for vector types.

       InfinityMetric is a metric of the Euclidean n-Plane. The distance
       between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ max_i |p_i - q_i|\f$

       @relates Metric
    */
    template< class T > class InfinityMetric : public Metric< T >
    {
      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::normInf (q);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::distInf (a, b);
        }
    };

    /**
       @brief Weighted infinity norm metric for vector types.

       Given a vector of weights \f$\mathbf{\omega}\in\mathbb{R}^n\f$, the
       distance between two points
       \f$ P = (p_1, p_2, ..., p_n) \f$
       and
       \f$ Q = (q_1, q_2, ..., q_n) \f$
       is defined as
       \f$ max_i |\omega_i * (p_i - q_i)|\f$

       @relates Metric
    */
    template< class T > class WeightedInfinityMetric : public Metric< T >
    {
      public:
        /**
           @brief Weighted metric.
           @param weights [in] Weights for the metric.
        */
        WeightedInfinityMetric (const typename Metric< T >::value_type& weights) :
            _weights (weights)
        {}

      protected:
        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& q) const
        {
            return MetricUtil::normInfWeighted (q, _weights);
        }

        typename Metric< T >::scalar_type
        doDistance (const typename Metric< T >::value_type& a,
                    const typename Metric< T >::value_type& b) const
        {
            return MetricUtil::distInfWeighted (a, b, _weights);
        }

        int doSize () const { return (int) _weights.size (); }

        const typename Metric< T >::value_type _weights;
    };

    /**
       @brief Mahalanobis distance metric for vector types.

       The Mahalonabis distance between two vectors \f$\mathbf{a}\f$
       and \f$\mathbf{b}\f$ both in \f$\mathbb{R}^n\f$ are defined as
       \f$d=\sqrt{(\mathbf{a}-\mathbf{b})^T \mathbf{\Omega} (\mathbf{a}-\mathbf{b})}\f$
       where \f$\mathbf{\Omega}\in \mathbb{R}^{n\times n}\f$.
    */
    template< class T > class MahalanobisMetric : public Metric< T >
    {
      private:
        typedef typename Metric< T >::value_type value_type;
        typedef typename Metric< T >::scalar_type scalar_type;
        typedef Eigen::Matrix< value_type, Eigen::Dynamic, Eigen::Dynamic > BaseM;
        typedef Eigen::Matrix< value_type, Eigen::Dynamic, 1 > BaseV;
        BaseM _omega;

      public:
        /**
         * @brief Constructs Mahalanobis metric object with the specified
         * weights.
         * @param omega [in] the weights \f$\mathbf{\Omega}\f$
         */
        MahalanobisMetric (const BaseM& omega) : _omega (omega) {}

      private:
        scalar_type norm (const BaseV& vec) const { return sqrt (vec.dot (_omega * vec)); }

        scalar_type doDistance (const value_type& q) const
        {
            BaseV vec (q.size ());
            for (size_t i = 0; i < q.size (); i++)
                vec[i] = q[i];

            return norm (vec);
        }

        typename Metric< T >::scalar_type doDistance (const value_type& a,
                                                      const value_type& b) const
        {
            BaseV vec (a.size ());
            for (size_t i = 0; i < a.size (); i++)
                vec[i] = a[i] - b[i];

            return norm (vec);
        }

        int doSize () const { return _omega.size1 (); }
    };

    /**
     * @brief a distance metric over rotations. The distance between two rotations
     * is the smalles angle that rotates the one into the other.
     */
    template< class T > class Rotation3DAngleMetric : public Metric< rw::math::Rotation3D< T > >
    {
      protected:
        T doDistance (const rw::math::Rotation3D< T >& r) const
        {
            EAA< T > eaa (r);
            return eaa.angle ();
        }

        T doDistance (const rw::math::Rotation3D< T >& a, const rw::math::Rotation3D< T >& b) const
        {
            return doDistance (a * inverse (b));
        }
    };

    /**
     * @brief distance metrics between points in SE3.
     */
    template< class T > class Transform3DAngleMetric : public Metric< rw::math::Transform3D< T > >
    {
      public:
        Transform3DAngleMetric (T posWeight, T angWeight) :
            _posWeight (posWeight), _angWeight (angWeight)
        {}

      protected:
        T doDistance (const rw::math::Transform3D< T >& t) const
        {
            EAA< T > eaa (t.R ());
            const T ang = eaa.angle ();
            const T pos = t.P ().norm2 ();
            return pos * _posWeight + ang * _angWeight;
        }

        T doDistance (const rw::math::Transform3D< T >& a,
                      const rw::math::Transform3D< T >& b) const
        {
            return doDistance (inverse (b) * a);
        }

      private:
        T _posWeight;
        T _angWeight;
    };

    /**
       @brief Metric constructor functions.

       The constructor functions are parameterized by a type of vector. Valid
       vector types include:

       - Q
       - Vector2D<double>
       - Vector3D<double>
       - std::vector<double>
       - Eigen::VectorXd
    */
    class MetricFactory
    {
      public:
        /**
           @brief Euclidean configuration metric.

           See class EuclideanMetric for details.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr makeEuclidean ()
        {
            return rw::core::ownedPtr (new EuclideanMetric< VectorType >);
        }

        /**
           @brief Weighted Euclidean configuration metric.

           See class WeightedEuclideanMetric for details.

           @param weights [in] Weights for the metric.
           @return Weighted Euclidean metric.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr
        makeWeightedEuclidean (const VectorType& weights)
        {
            return rw::core::ownedPtr (new WeightedEuclideanMetric< VectorType > (weights));
        }

        /**
           @brief Infinity configuration metric.

           See class InfinityMetric for details.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr makeInfinity ()
        {
            return rw::core::ownedPtr (new InfinityMetric< VectorType >);
        }

        /**
           @brief Weighted infinity configuration metric.

           See class WeightedInfinity for details.

           @param weights [in] Weights for the metric.
           @return Weighted infinity metric.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr
        makeWeightedInfinity (const VectorType& weights)
        {
            return rw::core::ownedPtr (new WeightedInfinityMetric< VectorType > (weights));
        }

        /**
           @brief Mahalanobis configuration metric.

           See class MahalanobisMetric for details.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr makeMahalanobis (
            const Eigen::Matrix< typename VectorType::value_type, Eigen::Dynamic, 1 >& omega)
        {
            return rw::core::ownedPtr (new MahalanobisMetric< VectorType > (omega));
        }

        /**
           @brief Manhattan configuration metric.

           See class ManhattanMetric for details.
        */
        template< class VectorType >
        inline static typename Metric< VectorType >::Ptr makeManhattan ()
        {
            return rw::core::ownedPtr (new ManhattanMetric< VectorType >);
        }

        /**
           @brief WeightedManhattan configuration metric.

           See class WeightedManhattanMetric for details.
        */
        template< class VectorType >
        inline static typename rw::core::Ptr<rw::math::Metric< VectorType >>
        makeWeightedManhattan (const VectorType& weights)
        {
            return rw::core::ownedPtr (new WeightedManhattanMetric< VectorType > (weights));
        }

        /**
         * @brief Metric computing distance between two rotations.
         *
         * The metric is defined as the angle of the rw::math::EAA
         * of the rotation.
         */
        template< class T >
        static typename rw::core::Ptr< rw::math::Metric< rw::math::Rotation3D< T > > >
        makeRotation3DMetric ()
        {
            return rw::core::ownedPtr (new Rotation3DAngleMetric< T > ());
        }

        /**
         * @brief Metric computing distance between two transformations
         *
         * The metric is defined as a weighted sum of the positional distance and the
         * angle of the rw::math::EAA of the rotation.
         *
         * @param linWeight [in] Positional weight.
         * @param angWeight [in] Angular weight.
         */
        template< class T >
        static typename rw::core::Ptr< rw::math::Metric< rw::math::Transform3D< T > > >
        makeTransform3DMetric (double linWeight, double angWeight)
        {
            return rw::core::ownedPtr (new Transform3DAngleMetric< T > (linWeight, angWeight));
        }

      private:    // The constructers are declared private so that this class can't be instanciated
                  // and avoid auto generated constucters
        MetricFactory ();
        MetricFactory (const MetricFactory&);
        MetricFactory& operator= (const MetricFactory&);
    };
#if !defined(SWIG)
    extern template class rw::math::ManhattanMetric< Q >;
    extern template class rw::math::WeightedManhattanMetric< Q >;

    extern template class rw::math::EuclideanMetric< Q >;
    extern template class rw::math::WeightedEuclideanMetric< Q >;

    extern template class rw::math::InfinityMetric< Q >;
    extern template class rw::math::WeightedInfinityMetric< Q >;

    extern template class rw::math::ManhattanMetric< Vector2D<> >;
    extern template class rw::math::WeightedManhattanMetric< Vector2D<> >;

    extern template class rw::math::EuclideanMetric< Vector2D<> >;
    extern template class rw::math::WeightedEuclideanMetric< Vector2D<> >;

    extern template class rw::math::InfinityMetric< Vector2D<> >;
    extern template class rw::math::WeightedInfinityMetric< Vector2D<> >;

    extern template class rw::math::ManhattanMetric< Vector2D< float > >;
    extern template class rw::math::WeightedManhattanMetric< Vector2D< float > >;

    extern template class rw::math::EuclideanMetric< Vector2D< float > >;
    extern template class rw::math::WeightedEuclideanMetric< Vector2D< float > >;

    extern template class rw::math::InfinityMetric< Vector2D< float > >;
    extern template class rw::math::WeightedInfinityMetric< Vector2D< float > >;

    extern template class rw::math::ManhattanMetric< Vector3D<> >;
    extern template class rw::math::WeightedManhattanMetric< Vector3D<> >;

    extern template class rw::math::EuclideanMetric< Vector3D<> >;
    extern template class rw::math::WeightedEuclideanMetric< Vector3D<> >;

    extern template class rw::math::InfinityMetric< Vector3D<> >;
    extern template class rw::math::WeightedInfinityMetric< Vector3D<> >;

    extern template class rw::math::ManhattanMetric< Vector3D< float > >;
    extern template class rw::math::WeightedManhattanMetric< Vector3D< float > >;

    extern template class rw::math::EuclideanMetric< Vector3D< float > >;
    extern template class rw::math::WeightedEuclideanMetric< Vector3D< float > >;

    extern template class rw::math::InfinityMetric< Vector3D< float > >;
    extern template class rw::math::WeightedInfinityMetric< Vector3D< float > >;

    extern template class rw::math::ManhattanMetric< Eigen::VectorXd >;
    extern template class rw::math::WeightedManhattanMetric< Eigen::VectorXd >;

    extern template class rw::math::EuclideanMetric< Eigen::VectorXd >;
    extern template class rw::math::WeightedEuclideanMetric< Eigen::VectorXd >;

    extern template class rw::math::InfinityMetric< Eigen::VectorXd >;
    extern template class rw::math::WeightedInfinityMetric< Eigen::VectorXd >;

    extern template class rw::math::ManhattanMetric< Eigen::VectorXf >;
    extern template class rw::math::WeightedManhattanMetric< Eigen::VectorXf >;

    extern template class rw::math::EuclideanMetric< Eigen::VectorXf >;
    extern template class rw::math::WeightedEuclideanMetric< Eigen::VectorXf >;

    extern template class rw::math::InfinityMetric< Eigen::VectorXf >;
    extern template class rw::math::WeightedInfinityMetric< Eigen::VectorXf >;

    extern template class rw::math::ManhattanMetric< std::vector< double > >;
    extern template class rw::math::WeightedManhattanMetric< std::vector< double > >;

    extern template class rw::math::EuclideanMetric< std::vector< double > >;
    extern template class rw::math::WeightedEuclideanMetric< std::vector< double > >;

    extern template class rw::math::InfinityMetric< std::vector< double > >;
    extern template class rw::math::WeightedInfinityMetric< std::vector< double > >;

    extern template class rw::math::ManhattanMetric< std::vector< float > >;
    extern template class rw::math::WeightedManhattanMetric< std::vector< float > >;

    extern template class rw::math::EuclideanMetric< std::vector< float > >;
    extern template class rw::math::WeightedEuclideanMetric< std::vector< float > >;

    extern template class rw::math::InfinityMetric< std::vector< float > >;
    extern template class rw::math::WeightedInfinityMetric< std::vector< float > >;
#endif
    /* @} */
}}    // namespace rw::math

#endif    // end include guard
