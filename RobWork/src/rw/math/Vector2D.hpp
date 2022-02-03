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

#ifndef RW_MATH_VECTOR2D_HPP
#define RW_MATH_VECTOR2D_HPP

/**
 * @file Vector2D.hpp
 */

#if !defined(SWIG)
#include <rw/common/Serializable.hpp>

#include <Eigen/Core>
#endif

namespace rw { namespace math {
    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 2D vector @f$ \mathbf{v}\in \mathbb{R}^2 @f$
     *
     * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
     *  \begin{array}{c}
     *  v_x \\
     *  v_y
     *  \end{array}
     *  \right]
     *  @f$
     *
     *  In addition, Vector2D supports the cross product operator:
     *  v3 = cross(v1, v2)
     *
     *  Usage example:
     *  @code
     *  using namespace rw::math;
     *
     *  Vector2D<> v1(1.0, 2.0);
     *  Vector2D<> v2(6.0, 7.0);
     *  Vector2D<> v3 = cross( v1, v2 );
     *  Vector2D<> v4 = v2 - v1;
     *  @endcode
     */
    template< class T = double > class Vector2D
    {
      public:
        //! Eigen based Vector2D
        typedef Eigen::Matrix< T, 2, 1 > EigenVector2D;

        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a 2D vector initialized with 0's
         */
        Vector2D ()
        {
            _vec[0] = 0;
            _vec[1] = 0;
        }

        /**
         * @brief Creates a 2D vector
         *
         * @param x [in] @f$ x @f$
         *
         * @param y [in] @f$ y @f$
         */
        Vector2D (T x, T y)
        {
            _vec[0] = x;
            _vec[1] = y;
        }

        /**
         * @brief Creates a 2D vector from Eigen Vector
         * @param r [in] an Eigen Vector
         */
        template< class R > Vector2D (const Eigen::MatrixBase< R >& r)
        {
            EigenVector2D v (r);
            _vec[0] = v (0);
            _vec[1] = v (1);
        }

        /**
         * @brief Copy Constructor
         */
        Vector2D (const Vector2D< T >& copy)
        {
            _vec[0] = copy[0];
            _vec[1] = copy[1];
        }

        /**
           @brief Returns Eigen vector equivalent to *this.
         */
        EigenVector2D e () const
        {
            EigenVector2D v;
            v (0) = _vec[0];
            v (1) = _vec[1];
            return v;
        }

        /**
           @brief The dimension of the vector (i.e. 2).

           This method is provided to help support generic algorithms using
           size() and operator[].
        */
        size_t size () const { return 2; }

        // Various operators.
#if !defined(SWIG)
        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return const reference to element
         */
        const T& operator() (size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return reference to element
         */
        T& operator() (size_t i) { return _vec[i]; }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator[] (size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator[] (size_t i) { return _vec[i]; }
#else
        ARRAYOPERATOR (T);
#endif

        /**
           @brief Scalar division.
         */
        const Vector2D< T > operator/ (T s) const
        {
            return Vector2D< T > ((*this)[0] / s, (*this)[1] / s);
        }

        /**
           @brief Scalar multiplication.
         */
        const Vector2D< T > operator* (T s) const
        {
            return Vector2D< T > ((*this)[0] * s, (*this)[1] * s);
        }
#if !defined(SWIGPYTHON)
        /**
           @brief Scalar multiplication.
         */
        friend const Vector2D< T > operator* (T s, const Vector2D< T >& v)
        {
            return Vector2D< T > (s * v[0], s * v[1]);
        }
#endif

        /**
           @brief Vector subtraction.
         */
        const Vector2D< T > operator- (const Vector2D< T >& b) const
        {
            return Vector2D< T > ((*this) (0) - b (0), (*this) (1) - b (1));
        }

        /**
           @brief Vector addition.
         */
        const Vector2D< T > operator+ (const Vector2D< T >& b) const
        {
            return Vector2D< T > ((*this) (0) + b (0), (*this) (1) + b (1));
        }

        /**
           @brief Scalar multiplication.
         */
        Vector2D< T >& operator*= (T s)
        {
            _vec[0] *= s;
            _vec[1] *= s;
            return *this;
        }

        /**
           @brief Scalar division.
         */
        Vector2D< T >& operator/= (T s)
        {
            _vec[0] /= s;
            _vec[1] /= s;
            return *this;
        }

        /**
           @brief Vector addition.
         */
        Vector2D< T >& operator+= (const Vector2D< T >& v)
        {
            _vec[0] += v (0);
            _vec[1] += v (1);
            return *this;
        }

        /**
           @brief Vector subtraction.
         */
        Vector2D< T >& operator-= (const Vector2D< T >& v)
        {
            _vec[0] -= v (0);
            _vec[1] -= v (1);
            return *this;
        }

        /**
           @brief Unary minus.
         */
        const Vector2D< T > operator- () const { return Vector2D< T > (-_vec[0], -_vec[1]); }

        /**
         * @brief Compares \b a and \b b for equality.
         * @param b [in]
         * @return True if a equals b, false otherwise.
         */
        bool operator== (const Vector2D< T >& b) const { return _vec[0] == b[0] && _vec[1] == b[1]; }

        /**
         *  @brief Compares \b a and \b b for inequality.
         * @param b [in]
         * @return True if a and b are different, false otherwise.
         */
        bool operator!= (const Vector2D< T >& b) const { return !(*this == b); }

        /**
         * @brief returns the counter clock-wise angle between
         * this vector and the x-axis vector (1,0). The angle
         * returned will be in the interval [-Pi,Pi]
         */
        double angle () { return atan2 (_vec[1], _vec[0]); }
#if !defined(SWIG)
        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<< (std::ostream& out, const Vector2D< T >& v)
        {
            return out << "Vector2D {" << v[0] << ", " << v[1] << "}";
        }
#else
        TOSTRING (rw::math::Vector2D< T >);
#endif

        /**
         * @brief Returns the Euclidean norm (2-norm) of the vector
         * @return the norm
         */
        T norm2 () const { return sqrt (_vec[0] * _vec[0] + _vec[1] * _vec[1]); }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the vector
         * @return the norm
         */
        T norm1 () const { return fabs (_vec[0]) + fabs (_vec[1]); }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
         * @return the norm
         */
        T normInf () const
        {
            T res      = fabs (_vec[0]);
            const T f1 = fabs (_vec[1]);
            if (f1 > res)
                res = f1;
            return res;
        }

      private:
        T _vec[2];
    };

    /**
     * @brief Calculates the 2D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * @param v1 [in] @f$ \mathbf{v1} @f$
     *
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 2D vector cross product is defined as:
     *
     * @f$
     * \mathbf{v1} \times \mathbf{v2} =  v1_x * v2_y - v1_y * v2_x
     * @f$
     */
    template< class T > T cross (const Vector2D< T >& v1, const Vector2D< T >& v2)
    {
        return v1 (0) * v2 (1) - v1 (1) * v2 (0);
    }

    /**
     * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     */
    template< class T > double dot (const Vector2D< T >& v1, const Vector2D< T >& v2)
    {
        return v1 (0) * v2 (0) + v1 (1) * v2 (1);
    }

    /**
     * @brief calculates the counter clock-wise angle from v1 to
     * v2. the value returned will be in the interval [-2Pi,2Pi]
     */
    template< class T > double angle (const Vector2D< T >& v1, const Vector2D< T >& v2)
    {
        return atan2 (v2 (1), v2 (0)) - atan2 (v1 (1), v1 (0));
    }

    /**
     * @brief Returns the normalized vector
     * \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
     *
     * If \f$ \| \mathbf{v} \| = 0\f$ then the zero vector is returned.
     *
     * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
     *
     * @return the normalized vector \f$ \mathbf{n} \f$
     */
    template< class T > const Vector2D< T > normalize (const Vector2D< T >& v)
    {
        T length = v.norm2 ();
        if (length != 0)
            return Vector2D< T > (v (0) / length, v (1) / length);
        else
            return Vector2D< T > (0, 0);
    }

    /**
     * @brief Casts Vector2D<T> to Vector2D<Q>
     *
     * @param v [in] Vector2D with type T
     *
     * @return Vector2D with type Q
     */
    template< class Q, class T > const Vector2D< Q > cast (const Vector2D< T >& v)
    {
        return Vector2D< Q > (static_cast< Q > (v (0)), static_cast< Q > (v (1)));
    }

#if !defined(SWIG)
    extern template class rw::math::Vector2D< double >;
    extern template class rw::math::Vector2D< float >;
#else
    SWIG_DECLARE_TEMPLATE (Vector2Dd, rw::math::Vector2D< double >);
    SWIG_DECLARE_TEMPLATE (Vector2Df, rw::math::Vector2D< float >);
#endif
    using Vector2Dd = Vector2D< double >;
    using Vector2Df = Vector2D< float >;

    /**@}*/
}}    // namespace rw::math

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Vector2D
         */
        template<>
        void write (const rw::math::Vector2D< double >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Vector2D
         */
        template<>
        void write (const rw::math::Vector2D< float >& sobject, rw::common::OutputArchive& oarchive,
                    const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Vector2D
         */
        template<>
        void read (rw::math::Vector2D< double >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Vector2D
         */
        template<>
        void read (rw::math::Vector2D< float >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

namespace boost { namespace serialization {
    /**
     * @brief Boost serialization.
     * @param archive [in] the boost archive to read from or write to.
     * @param vector [in/out] the vector to read/write.
     * @param version [in] class version (currently version 0).
     * @relatedalso rw::math::Vector2D
     */
    template< class Archive, class T >
    void serialize (Archive& archive, rw::math::Vector2D< T >& vector, const unsigned int version)
    {
        archive& vector[0];
        archive& vector[1];
    }
}}    // namespace boost::serialization

#endif    // end include guard
