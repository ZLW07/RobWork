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

#ifndef RW_MATH_VECTOR3D_HPP
#define RW_MATH_VECTOR3D_HPP

/**
 * @file Vector3D.hpp
 */

#if !defined(SWIG)
#include <rw/common/Serializable.hpp>

#include <Eigen/Eigen>
#endif
namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 3D vector @f$ \mathbf{v}\in \mathbb{R}^3 @f$
     *
     * @f$ \robabx{i}{j}{\mathbf{v}} = \left[
     *  \begin{array}{c}
     *  v_x \\
     *  v_y \\
     *  v_z
     *  \end{array}
     *  \right]
     *  @f$
     *
     *  Usage example:
     *
     *  \code
     *  const Vector3D<> v1(1.0, 2.0, 3.0);
     *  const Vector3D<> v2(6.0, 7.0, 8.0);
     *  const Vector3D<> v3 = cross(v1, v2);
     *  const double d = dot(v1, v2);
     *  const Vector3D<> v4 = v2 - v1;
     *  \endcode
     */
    template< class T = double > class Vector3D
    {
      public:
        //! Eigen type equivalent to Vector3D
        typedef Eigen::Matrix< T, 3, 1 > EigenVector3D;

        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a 3D vector initialized with 0's
         */
        Vector3D ()
        {
            _vec[0] = 0;
            _vec[1] = 0;
            _vec[2] = 0;
        }

        /**
         * @brief Creates a 3D vector
         * @param x [in] @f$ x @f$
         * @param y [in] @f$ y @f$
         * @param z [in] @f$ z @f$
         */
        Vector3D (T x, T y, T z)
        {
            _vec[0] = x;
            _vec[1] = y;
            _vec[2] = z;
        }
        
        /**
         * @brief Copy constructor
         * @param vec [in] vector to copy
         */
        Vector3D (const Vector3D<T>& copy_vec): _vec(copy_vec._vec)
        {
        }

        /**
         * @brief Creates a 3D vector from vector_expression
         * @param r [in] an Eigen Vector
         */
        template< class R > explicit Vector3D (const Eigen::MatrixBase< R >& r)
        {
            _vec[0] = T( r.row (0) (0));
            _vec[1] = T( r.row (1) (0));
            _vec[2] = T( r.row (2) (0));
        }

        /**
         * @brief construct vector from std::vector
         * @param vec [in] the vector to construct from
         */
        Vector3D (const std::vector< T >& vec)
        {
            if (vec.size () != 3u) {
                RW_THROW ("Wrong Size vector matrix: N of size:" << 3 << " and vector of size: "
                                                                 << vec.size () << "given");
            }
            for (size_t i = 0; i < 3u; i++) {
                _vec[i] = vec[i];
            }
        }

        /**
         *  @brief The dimension of the vector (i.e. 3).
         * This method is provided to help support generic algorithms using
         * size() and operator[].
         * @return the size
         */
        size_t size () const { return 3u; }

        /**
         * @brief Get zero vector.
         * @return vector.
         */
        static Vector3D< T > zero () { return Vector3D< T > (0, 0, 0); }

        /**
         * @brief Get x vector (1,0,0)
         * @return vector.
         */
        static Vector3D< T > x () { return Vector3D< T > (1.0, 0, 0); }

        /**
         * @brief Get y vector (0,1,0)
         * @return vector.
         */
        static Vector3D< T > y () { return Vector3D< T > (0, 1.0, 0); }

        /**
         * @brief Get z vector (0,0,1)
         * @return vector.
         */
        static Vector3D< T > z () { return Vector3D< T > (0, 0, 1.0); }

        // ###################################################
        // #                 Math Operations                 #
        // ###################################################

        // ########## Eigen Operations

        /**
         * @brief element wise division.
         * @param rhs [in] the vector being devided with
         * @return the resulting Vector3D
         */
        template< class R > Vector3D< T > elemDivide (const Eigen::MatrixBase< R >& rhs) const
        {
            Vector3D< T > ret = *this;
            for (size_t i = 0; i < size (); i++) {
                ret._vec[i] /= rhs[i];
            }
            return ret;
        }

        /**
         * @brief Elementweise multiplication.
         * @param rhs [in] vector
         * @return the element wise product
         */
        template< class R > Vector3D< T > elemMultiply (const Eigen::MatrixBase< R >& rhs) const
        {
            Vector3D< T > ret = *this;
            for (size_t i = 0; i < size (); i++) {
                ret._vec[i] *= rhs[i];
            }
            return ret;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > Vector3D< T > operator- (const Eigen::MatrixBase< R >& rhs) const
        {
            return Vector3D< T > (_vec - rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend Vector3D< T > operator- (const Eigen::MatrixBase< R >& lhs, const Vector3D< T >& rhs)
        {
            return Vector3D< T > (lhs - rhs.e ());
        }

        /**
         * @brief Vector addition.
         */
        template< class R > Vector3D< T > operator+ (const Eigen::MatrixBase< R >& rhs) const
        {
            return Vector3D< T > (_vec + rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend Vector3D< T > operator+ (const Eigen::MatrixBase< R >& lhs, const Vector3D< T >& rhs)
        {
            return Vector3D< T > (lhs + rhs.e ());
        }

        // ########## Vector3D Operations

        /**
         * @brief element wise division.
         * @param rhs [in] the vector being devided with
         * @return the resulting Vector3D
         */
        Vector3D< T > elemDivide (const Vector3D< T >& rhs) const
        {
            return Vector3D< T > (
                _vec[0] / rhs._vec[0], _vec[1] / rhs._vec[1], _vec[2] / rhs._vec[2]);
        }

        /**
         * @brief Elementweise multiplication.
         * @param rhs [in] vector
         * @return the element wise product
         */
        Vector3D< T > elemMultiply (const Vector3D< T >& rhs) const
        {
            return Vector3D< T > (
                _vec[0] * rhs._vec[0], _vec[1] * rhs._vec[1], _vec[2] * rhs._vec[2]);
        }

        /**
         * @brief Vector subtraction.
         */
        Vector3D< T > operator- (const Vector3D< T >& b) const
        {
            return Vector3D< T > (_vec[0] - b[0], _vec[1] - b[1], _vec[2] - b[2]);
        }

        /**
         * @brief Vector addition.
         */
        Vector3D< T > operator+ (const Vector3D< T >& b) const
        {
            return Vector3D< T > (_vec[0] + b[0], _vec[1] + b[1], _vec[2] + b[2]);
        }

        /**
         * @brief Unary minus.
         * @brief negative version
         */
        Vector3D< T > operator- () const { return Vector3D< T > (-_vec[0], -_vec[1], -_vec[2]); }

        // ########## Scalar Operations

        /**
         * @brief Scalar division.
         * @param s [in] the scalar to devide with
         * @return result of devision
         */
        Vector3D< T > operator/ (T s) const
        {
            return Vector3D< T > (_vec[0] / s, _vec[1] / s, _vec[2] / s);
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar division.
         * @param lhs [in] the scalar to devide with
         * @param rhs [out] the vector beind devided
         * @return result of devision
         */
        friend Vector3D< T > operator/ (T lhs, const Vector3D< T >& rhs)
        {
            return Vector3D< T > (lhs / rhs._vec[0], lhs / rhs._vec[1], lhs / rhs._vec[2]);
        }
#endif
        /**
         * @brief Scalar multiplication.
         * @param rhs [in] the scalar to multiply with
         * @return the product
         */
        Vector3D< T > operator* (T rhs) const
        {
            return Vector3D< T > (_vec[0] * rhs, _vec[1] * rhs, _vec[2] * rhs);
        }

        /**
         * @brief Scalar multiplication.
         * @param lhs [in] the scalar to multiply with
         * @param rhs [in] the Vector to be multiplied
         * @return the product
         */
        friend Vector3D< T > operator* (T lhs, const Vector3D< T >& rhs)
        {
            return Vector3D< T > (lhs * rhs[0], lhs * rhs[1], lhs * rhs[2]);
        }

        /**
         * @brief Scalar multiplication.
         * @param rhs [in] the Eigen vector^T or matrix to multiply with
         * @return the product
         */
        template< class R > Vector3D< T > operator* (const Eigen::MatrixBase< R >& rhs) const
        {
            return Vector3D< T > (this->e()*rhs);
        }

        /**
         * @brief Scalar multiplication.
         * @param lhs [in] the Eigen vector^T or matrix to multiply with
         * @param rhs [in] the Vector to be multiplied
         * @return the product
         */
        template< class R > friend Vector3D< T > operator* (const Eigen::MatrixBase< R >& lhs, const Vector3D< T >& rhs)
        {
            return Vector3D< T > (lhs*rhs.e());
        }

        /**
         * @brief Scalar subtraction.
         */
        Vector3D< T > elemSubtract (const T rhs) const
        {
            return Vector3D< T > (_vec[0] - rhs, _vec[1] - rhs, _vec[2] - rhs);
        }

        /**
         * @brief Scalar addition.
         */
        Vector3D< T > elemAdd (const T rhs) const
        {
            return Vector3D< T > (_vec[0] + rhs, _vec[1] + rhs, _vec[2] + rhs);
        }

        // ########### Math Functions

        /**
         * @brief Returns the Euclidean norm (2-norm) of the vector
         * @return the norm
         */
        T norm2 () const
        {
            return sqrt (_vec[0] * _vec[0] + _vec[1] * _vec[1] + _vec[2] * _vec[2]);
        }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the vector
         * @return the norm
         */
        T norm1 () const { return fabs (_vec[0]) + fabs (_vec[1]) + fabs (_vec[2]); }

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
            const T f2 = fabs (_vec[2]);
            if (f2 > res)
                res = f2;
            return res;
        }

        /**
         * @brief Calculate cross product
         * @param vec [in] the vector to cross with
         * @return the cross product
         */
        Vector3D< T > cross (const Vector3D& vec) const
        {
            return Vector3D< T > (_vec.cross (vec._vec));
        }

        /**
         * @brief calculate the dot product
         * @param vec [in] the vecor to be dotted
         * @return the dot product
         */
        T dot (const Vector3D& vec) const { return _vec.dot (vec._vec); }

        /**
         * @brief normalize vector to get length 1
         * @return the normalized Vector
         */
        Vector3D< T > normalize ()
        {
            T length = norm2 ();
            if (length != 0)
                return (*this) / length;
            else
                return Vector3D< T > (0, 0, 0);
        }

        // ###################################################
        // #                Acces Operators                  #
        // ###################################################

        /**
         * @brief Returns Reference to Eigen Vector
         * @return reference to underling eigen
         */
        EigenVector3D& e () { return _vec; }

        /**
         * @brief Returns Reference to Eigen Vector
         * @return copy of eigen vector
         */
        const EigenVector3D e () const { return _vec; }

#if !defined(SWIG)
        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator() (size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
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
#if !defined(SWIG)
        /**
         * @brief Streaming operator.
         * @param out [in/out] the stream to continue
         * @param v [in] the vector to stream
         * @param reference to \b out
         */
        friend std::ostream& operator<< (std::ostream& out, const Vector3D< T >& v)
        {
            return out << "Vector3D(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
        }
#else
        TOSTRING (rw::math::Vector3D< T >);
#endif
        // ###################################################
        // #             assignement Operators               #
        // ###################################################

        /**
         * @brief Scalar multiplication.
         */
        Vector3D< T >& operator*= (T s)
        {
            _vec[0] *= s;
            _vec[1] *= s;
            _vec[2] *= s;
            return *this;
        }

        /**
         * @brief Scalar division.
         */
        Vector3D< T >& operator/= (T s)
        {
            _vec[0] /= s;
            _vec[1] /= s;
            _vec[2] /= s;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        Vector3D< T >& operator+= (const Vector3D< T >& v)
        {
            _vec[0] += v._vec[0];
            _vec[1] += v._vec[1];
            _vec[2] += v._vec[2];
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        Vector3D< T >& operator-= (const Vector3D< T >& v)
        {
            _vec[0] -= v._vec[0];
            _vec[1] -= v._vec[1];
            _vec[2] -= v._vec[2];
            return *this;
        }

        /**
         * @brief copy a vector from eigen type
         * @param r [in] an Eigen Vector
         */
        template< class R > Vector3D< T >& operator= (const Eigen::MatrixBase< R >& r)
        {
            _vec = r;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        template< class R > Vector3D< T >& operator+= (const Eigen::MatrixBase< R >& r)
        {
            _vec += r;
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > Vector3D< T >& operator-= (const Eigen::MatrixBase< R >& r)
        {
            _vec -= r;
            return *this;
        }

        // ###################################################
        // #                    Comparetors                  #
        // ###################################################

        /**
         * @brief Compare with \b b for equality.
         * @param b [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        bool operator== (const Vector3D< T >& b) const
        {
            return _vec[0] == b[0] && _vec[1] == b[1] && _vec[2] == b[2];
        }

        /**
           @brief Compare with \b b for inequality.
           @param b [in] other vector.
           @return True if a and b are different, false otherwise.
        */
        bool operator!= (const Vector3D< T >& b) const { return !(*this == b); }

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        template< class R > bool operator== (const Eigen::MatrixBase< R >& rhs) const
        {
            return this->_vec == rhs;
        }

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        template< class R >
        friend bool operator== (const Eigen::MatrixBase< R >& lhs, const Vector3D< T >& rhs)
        {
            return lhs == rhs._vec;
        }

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param b [in] other vector.
         *  @return True if a and b are different, false otherwise.
         */
        template< class R > bool operator!= (const Eigen::MatrixBase< R >& rhs) const
        {
            return !(*this == rhs);
        }

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param b [in] other vector.
         *  @return True if a and b are different, false otherwise.
         */
        template< class R >
        friend bool operator!= (const Eigen::MatrixBase< R >& lhs, const Vector3D< T >& rhs)
        {
            return !(lhs == rhs);
        }
#if !defined(SWIG)
        /**
         * @brief implicit conversion to EigenVector
         */
        operator EigenVector3D () const { return this->e (); }

        /**
         * @brief implicit conversion to EigenVector
         */
        operator EigenVector3D& () { return this->e (); }
#endif

      private:
        EigenVector3D _vec;
    };

    /**
     * @brief Calculates the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D vector cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates Vector3D
     */
    template< class T > const Vector3D< T > cross (const Vector3D< T >& v1, const Vector3D< T >& v2)
    {
        return Vector3D< T > (v1[1] * v2[2] - v1[2] * v2[1],
                              v1[2] * v2[0] - v1[0] * v2[2],
                              v1[0] * v2[1] - v1[1] * v2[0]);
    }

    /**
     * @brief Calculates the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     * @param dst [out] the 3D vector cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D vector cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates Vector3D
     */
    template< class T >
    void cross (const Vector3D< T >& v1, const Vector3D< T >& v2, Vector3D< T >& dst)
    {
        dst[0] = v1[1] * v2[2] - v1[2] * v2[1];
        dst[1] = v1[2] * v2[0] - v1[0] * v2[2];
        dst[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }

    /**
     * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     *
     * @relates Vector3D
     */
    template< class T > T dot (const Vector3D< T >& v1, const Vector3D< T >& v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
        // return inner_prod(v1.m(), v2.m());
    }

    /**
     * @brief Returns the normalized vector \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
     * In case \f$ \|mathbf{v}\| = 0\f$ the zero vector is returned.
     * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
     * @return the normalized vector \f$ \mathbf{n} \f$
     *
     * @relates Vector3D
     */
    template< class T > const Vector3D< T > normalize (const Vector3D< T >& v)
    {
        T length = v.norm2 ();
        if (length != 0)
            return Vector3D< T > (v (0) / length, v (1) / length, v (2) / length);
        else
            return Vector3D< T > (0, 0, 0);
    }

    /**
     * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
     * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$ with n
     * determining the sign.
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     * @param n [in] @f$ \mathbf{n} @f$
     *
     * @return the angle
     *
     * @relates Vector3D
     */
    template< class T >
    double angle (const Vector3D< T >& v1, const Vector3D< T >& v2, const Vector3D< T >& n)
    {
        const Vector3D< T > nv1 = normalize (v1);
        const Vector3D< T > nv2 = normalize (v2);
        const Vector3D< T > nn  = normalize (n);
        return atan2 (dot (nn, cross (nv1, nv2)), dot (nv1, nv2));
    }

    /**
     * @brief Calculates the angle from @f$ \mathbf{v1}@f$ to @f$ \mathbf{v2} @f$
     * around the axis defined by @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the angle
     *
     * @relates Vector3D
     */
    template< class T > double angle (const Vector3D< T >& v1, const Vector3D< T >& v2)
    {
        Vector3D< T > n = cross (v1, v2);
        return angle (v1, v2, n);
    }

    /**
     * @brief Casts Vector3D<T> to Vector3D<Q>
     * @param v [in] Vector3D with type T
     * @return Vector3D with type Q
     *
     * @relates Vector3D
     */
    template< class Q, class T > const Vector3D< Q > cast (const Vector3D< T >& v)
    {
        return Vector3D< Q > (
            static_cast< Q > (v (0)), static_cast< Q > (v (1)), static_cast< Q > (v (2)));
    }
#if !defined(SWIG)
    extern template class rw::math::Vector3D< double >;
    extern template class rw::math::Vector3D< float >;
#else
    SWIG_DECLARE_TEMPLATE (Vector3Dd, rw::math::Vector3D< double >);
    SWIG_DECLARE_TEMPLATE (Vector3Df, rw::math::Vector3D< float >);
#endif

    using Vector3Dd = Vector3D< double >;
    using Vector3Df = Vector3D< float >;

    /**@}*/
}}    // namespace rw::math

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Vector3D
         */
        template<>
        void write (const rw::math::Vector3D< double >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Vector3D
         */
        template<>
        void write (const rw::math::Vector3D< float >& sobject, rw::common::OutputArchive& oarchive,
                    const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Vector3D
         */
        template<>
        void read (rw::math::Vector3D< double >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Vector3D
         */
        template<>
        void read (rw::math::Vector3D< float >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

namespace boost { namespace serialization {
    /**
     * @brief Boost serialization.
     * @param archive [in] the boost archive to read from or write to.
     * @param vector [in/out] the vector to read/write.
     * @param version [in] class version (currently version 0).
     * @relatedalso rw::math::Vector3D
     */
    template< class Archive, class T >
    void serialize (Archive& archive, rw::math::Vector3D< T >& vector, const unsigned int version)
    {
        archive& vector[0];
        archive& vector[1];
        archive& vector[2];
    }
}}    // namespace boost::serialization

#endif    // end include guard
