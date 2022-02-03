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

#ifndef RW_MATH_VectorND_HPP
#define RW_MATH_VectorND_HPP

/**
 * @file VectorND.hpp
 */
#if !defined(SWIG)
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rw/common/Serializable.hpp>
#include <rw/core/macros.hpp>

#include <Eigen/Core>
#endif

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A N-Dimensional Vector
     *
     */
    template< size_t N, class T = double > class VectorND : public rw::common::Serializable
    {
      public:
        //! The type of the internal Eigen Vector
        typedef Eigen::Matrix< T, N, 1 > EigenVectorND;

        //! Value type.
        typedef T value_type;

        /**
         * @brief Creates a N-dimensional VectorND
         */
        VectorND ()
        {
            if (N <= 0) {
                RW_THROW ("Vector to small, N must be larger than 0");
            }
            _vec = EigenVectorND (N);
        }

        /**
         * @brief Construct a Vector from N arguments
         * @param args [in] a list of arguments
         */
        template< typename... ARGS > VectorND (T arg0, ARGS... args)
        {
            if (N <= 0u) {
                RW_THROW ("Vector to small, N must be larger than 0");
            }
            size_t i = 1;
            ParamExpansion (i, args...);

            _vec[--i] = arg0;
        }

        /**
         * @brief construct vector from std::vector
         * @param vec [in] the vector to construct from
         */
        VectorND (const std::vector< T >& vec)
        {
            if (N <= 0u) {
                RW_THROW ("Vector to small, N must be larger than 0");
            }
            else if (vec.size () != N) {
                RW_THROW ("Wrong Size vector matrix: N of size:" << N << " and vector of size: "
                                                                 << vec.size () << "given");
            }
            for (size_t i = 0; i < N; i++) {
                _vec[i] = vec[i];
            }
        }

        /**
         * @brief Creates a 3D VectorND from Eigen type.
         *
         * @param v [in] an Eigen vector.
         */
        template< class R > VectorND (const Eigen::MatrixBase< R >& v)
        {
            if (v.cols () != 1 || v.rows () != N)
                RW_THROW ("Unable to initialize VectorND with " << v.rows () << " x " << v.cols ()
                                                                << " matrix");
            _vec = v;
        }

        /**
         * @brief The dimension of the VectorND (i.e. 3).
         * This method is provided to help support generic algorithms using
           size() and operator[].
         */
        size_t size () const { return N; }

        // ###################################################
        // #                 Math Operations                 #
        // ###################################################

        // ########## Eigen Operations

        /**
         * @brief element wise multiplication.
         * @param rhs [in] the vector being multiplied with
         * @return the resulting VectorND
         */
        template< class R > VectorND< N, T > elemMultiply (const Eigen::MatrixBase< R >& rhs) const
        {
            VectorND< N, T > ret = *this;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] *= rhs[i];
            }
            return ret;
        }

        /**
         * @brief element wise division.
         * @param lhs [in] vector
         * @param rhs [in] vector
         * @return the resulting VectorND
         */
        template< class R > VectorND< N, T > elemDivide (const Eigen::MatrixBase< R >& rhs) const
        {
            VectorND< N, T > ret;
            for (size_t i = 0; i < N; i++) {
                ret[i] = (*this)[i] / rhs[i];
            }
            return ret;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > VectorND< N, T > operator- (const Eigen::MatrixBase< R >& rhs) const
        {
            return VectorND< N, T > (_vec - rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend VectorND< N, T > operator- (const Eigen::MatrixBase< R >& lhs,
                                           const VectorND< N, T >& rhs)
        {
            return VectorND< N, T > (lhs - rhs.e ());
        }

        /**
         * @brief Vector addition.
         */
        template< class R > VectorND< N, T > operator+ (const Eigen::MatrixBase< R >& rhs) const
        {
            return VectorND< N, T > (_vec + rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend VectorND< N, T > operator+ (const Eigen::MatrixBase< R >& lhs,
                                           const VectorND< N, T >& rhs)
        {
            return VectorND< N, T > (lhs + rhs.e ());
        }

        // ########## VectorND Operations

        /**
         * @brief element wise division.
         * @param rhs [in] the vector being devided with
         * @return the resulting Vector3D
         */
        VectorND< N, T > elemDivide (const VectorND< N, T >& rhs) const
        {
            VectorND< N, T > ret = *this;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] /= rhs._vec[i];
            }
            return ret;
        }

        /**
         * @brief Elementweise multiplication.
         * @param rhs [in] vector
         * @return the element wise product
         */
        VectorND< N, T > elemMultiply (const VectorND< N, T >& rhs) const
        {
            VectorND< N, T > ret = *this;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] *= rhs._vec[i];
            }
            return ret;
        }

        /**
         * @brief Vector subtraction.
         */
        VectorND< N, T > operator- (const VectorND< N, T >& rhs) const
        {
            return VectorND< N, T > (_vec - rhs._vec);
        }

        /**
         * @brief Vector addition.
         */
        VectorND< N, T > operator+ (const VectorND< N, T >& rhs) const
        {
            return VectorND< N, T > (_vec + rhs._vec);
        }

        /**
         * @brief Unary minus.
         * @brief negative version
         */
        VectorND< N, T > operator- () const { return VectorND< N, T > (_vec * (-1)); }

        // ########## Scalar Operations

        /**
         * @brief Scalar division.
         * @param rhs [in] the scalar to devide with
         * @return result of devision
         */
        VectorND< N, T > operator/ (T rhs) const { return VectorND< N, T > (_vec / rhs); }

#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar division.
         * @param lhs [in] the scalar to devide with
         * @param rhs [out] the vector beind devided
         * @return result of devision
         */
        friend VectorND< N, T > operator/ (T lhs, const VectorND< N, T >& rhs)
        {
            VectorND< N, T > ret = rhs;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] = lhs / ret._vec[i];
            }
            return ret;
        }
#endif

        /**
         * @brief Scalar multiplication.
         * @param rhs [in] the scalar to multiply with
         * @return the product
         */
        VectorND< N, T > operator* (T rhs) const { return VectorND< N, T > (_vec * rhs); }

#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar multiplication.
         * @param lhs [in] the scalar to multiply with
         * @param rhs [in] the Vector to be multiplied
         * @return the product
         */
        friend VectorND< N, T > operator* (T lhs, const VectorND< N, T >& rhs)
        {
            return VectorND< N, T > (lhs * rhs._vec);
        }
#endif

        /**
         * @brief Scalar subtraction.
         */
        VectorND< N, T > elemSubtract (const T& rhs) const
        {
            VectorND< N, T > ret = *this;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] = ret._vec[i] - rhs;
            }
            return ret;
        }

        /**
         * @brief Scalar addition.
         */
        VectorND< N, T > elemAdd (const T& rhs) const
        {
            VectorND< N, T > ret = *this;
            for (size_t i = 0; i < N; i++) {
                ret._vec[i] = ret._vec[i] + rhs;
            }
            return ret;
        }

        // ########### Math Functions

        /**
         * @brief Returns the Euclidean norm (2-norm) of the VectorND
         * @return the norm
         */
        T norm2 () const { return _vec.norm (); }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the VectorND
         * @return the norm
         */
        T norm1 () const { return _vec.template lpNorm< 1 > (); }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the VectorND
         * @return the norm
         */
        T normInf () const { return _vec.template lpNorm< Eigen::Infinity > (); }

        /**
         * @brief calculate the dot product
         * @param vec [in] the vecor to be dotted
         * @return the dot product
         */
        double dot (const VectorND< N, T >& vec) const { return _vec.dot (vec._vec); }

        /**
         * @brief normalize vector to get length 1
         * @return the normalized Vector
         */
        VectorND< N, T > normalize ()
        {
            T length = norm2 ();
            if (length != 0)
                return (*this) / length;
            else
                return VectorND< N, T > ();
        }

        // ###################################################
        // #                Acces Operators                  #
        // ###################################################
#if !defined(SWIG)
        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator() (size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator() (size_t i) { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator[] (size_t i) const { return _vec[i]; }

        /**
         * @brief Returns reference to VectorND element
         * @param i [in] index in the VectorND \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator[] (size_t i) { return _vec[i]; }
#else
        ARRAYOPERATOR (T);
#endif
        /**
         * @brief Accessor for the internal Eigen VectorND.
         */
        EigenVectorND& e () { return _vec; }

        /**
           @brief Accessor for the internal Eigen VectorND.
         */
        const EigenVectorND& e () const { return _vec; }
#if !defined(SWIG)
        /**
         * @brief Streaming operator.
         * @param out [in/out] the stream to continue
         * @param v [in] the vector to stream
         * @param reference to \b out
         */
        friend std::ostream& operator<< (std::ostream& out, const VectorND< N, T >& v)
        {
            out << "Vector" << N << "D(";
            for (size_t i = 0; i < N - 1; i++) {
                out << v[i] << ", ";
            }
            out << v[N - 1] << ")";
            return out;
        }
#else
#define VECTORND(num, type) rw::math::VectorND< num, type >
        TOSTRING (VECTORND (N, T));
#undef VECTORND
#endif

        /**
         * @brief converts the vector to a std:vector
         * @return a std::vector
         */
        std::vector< T > toStdVector () const
        {
            std::vector< T > ret;
            for (size_t i = 0; i < N; i++) {
                ret.push_back (_vec[i]);
            }
            return ret;
        }

        // ###################################################
        // #             assignement Operators               #
        // ###################################################

        /**
         * @brief Scalar multiplication.
         */
        VectorND< N, T >& operator*= (double s)
        {
            _vec *= s;
            return *this;
        }

        /**
         * @brief Scalar division.
         */
        VectorND< N, T >& operator/= (double s)
        {
            _vec /= s;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        VectorND< N, T >& operator+= (const VectorND< N, T >& v)
        {
            _vec += v._vec;
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        VectorND< N, T >& operator-= (const VectorND< N, T >& v)
        {
            _vec -= v._vec;
            return *this;
        }

        /**
         * @brief copy a vector from eigen type
         * @param r [in] an Eigen Vector
         */
        template< class R > VectorND< N, T >& operator= (const Eigen::MatrixBase< R >& r)
        {
            _vec = r;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        template< class R > VectorND< N, T >& operator+= (const Eigen::MatrixBase< R >& r)
        {
            _vec += r;
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > VectorND< N, T >& operator-= (const Eigen::MatrixBase< R >& r)
        {
            _vec -= r;
            return *this;
        }

        // ###################################################
        // #                    Comparetors                  #
        // ###################################################
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
        friend bool operator== (const Eigen::MatrixBase< R >& lhs, const VectorND< N, T >& rhs)
        {
            return lhs == rhs._vec;
        }

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param ths [in] other vector.
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
        friend bool operator!= (const Eigen::MatrixBase< R >& lhs, const VectorND< N, T >& rhs)
        {
            return !(lhs == rhs);
        }

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        bool operator== (const VectorND< N, T >& rhs) const { return this->_vec == rhs._vec; }

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param rhs [in] other vector.
         *  @return True if a and b are different, false otherwise.
         */
        bool operator!= (const VectorND< N, T >& rhs) const { return !(*this == rhs); }

        // ###################################################
        // #                      OTHER                      #
        // ###################################################
#if !defined(SWIG)
        //! @copydoc rw::common::Serializable::write
        void write (rw::common::OutputArchive& oarchive, const std::string& id) const
        {
            oarchive.write (this->toStdVector (), id, "VectorND");
        }

        //! @copydoc rw::common::Serializable::read
        void read (rw::common::InputArchive& iarchive, const std::string& id)
        {
            std::vector< T > result (N, 0);
            iarchive.read (result, id, "VectorND");
            *this = VectorND< N, T > (result);
        }
#endif
#if !defined(SWIG)
        /**
         * @brief implicit conversion to EigenVector
         */
        operator EigenVectorND () const { return this->e (); }

        /**
         * @brief implicit conversion to EigenVector
         */
        operator EigenVectorND& () { return this->e (); }
#endif
        /**
         * @brief Get zero-initialized vector.
         * @return vector.
         */
        static VectorND< N, T > zero () { return Eigen::Matrix< T, N, 1 >::Zero (); }

      private:
        void ParamExpansion (size_t& i)
        {
            if (i > N) {
                RW_THROW ("Vector to big, argc(" << i << ") != N(" << N << ")");
            }
            else if (i < N) {
                RW_THROW ("Vector to small, argc(" << i << ") != N(" << N << ")");
            }
        }

        template< typename R > void ParamExpansion (size_t& i, R arg)
        {
            ParamExpansion (++i);
            _vec[--i] = T (arg);
        }

        template< typename R, typename... ARGS >
        void ParamExpansion (size_t& i, R arg, ARGS... args)
        {
            ParamExpansion (++i, args...);

            _vec[--i] = T (arg);
        }

        EigenVectorND _vec;
    };

    /**
     * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D VectorND cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates VectorND
     */
    template< size_t ND, class T >
    const VectorND< ND, T > cross (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2)
    {
        return v1.e ().cross (v2.e ());
        // return cross(v1.e(),v2.e());
    }

    /**
     * @brief Calculates the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     * @param dst [out] the 3D VectorND cross product @f$ \mathbf{v1} \times \mathbf{v2} @f$
     *
     * The 3D VectorND cross product is defined as:
     * @f$
     * \mathbf{v1} \times \mathbf{v2} = \left[\begin{array}{c}
     *  v1_y * v2_z - v1_z * v2_y \\
     *  v1_z * v2_x - v1_x * v2_z \\
     *  v1_x * v2_y - v1_y * v2_x
     * \end{array}\right]
     * @f$
     *
     * @relates VectorND
     */
    template< size_t ND, class T >
    void cross (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2, VectorND< ND, T >& dst)
    {
        dst = v1.e ().cross (v2.e ());
        // dst = cross(v1.m(),v2.m());
    }

    /**
     * @brief Calculates the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     * @param v1 [in] @f$ \mathbf{v1} @f$
     * @param v2 [in] @f$ \mathbf{v2} @f$
     *
     * @return the dot product @f$ \mathbf{v1} . \mathbf{v2} @f$
     *
     * @relates VectorND
     */
    template< size_t ND, class T > T dot (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2)
    {
        return v1.e ().dot (v2.e ());
    }

    /**
     * @brief Returns the normalized VectorND \f$\mathbf{n}=\frac{\mathbf{v}}{\|\mathbf{v}\|} \f$.
     * In case \f$ \|mathbf{v}\| = 0\f$ the zero VectorND is returned.
     * @param v [in] \f$ \mathbf{v} \f$ which should be normalized
     * @return the normalized VectorND \f$ \mathbf{n} \f$
     *
     * @relates VectorND
     */
    template< size_t ND, class T > const VectorND< ND, T > normalize (const VectorND< ND, T >& v)
    {
        // Create a copy
        VectorND< ND, T > res (v);
        res.e ().normalize ();
        return res;
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
     * @relates VectorND
     */
    template< size_t ND, class T >
    double angle (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2,
                  const VectorND< ND, T >& n)
    {
        const VectorND< ND, T > nv1 = normalize (v1);
        const VectorND< ND, T > nv2 = normalize (v2);
        const VectorND< ND, T > nn  = normalize (n);
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
     * @relates VectorND
     */
    template< size_t ND, class T >
    double angle (const VectorND< ND, T >& v1, const VectorND< ND, T >& v2)
    {
        VectorND< ND, T > n = cross (v1, v2);
        return angle (v1, v2, n);
    }

    /**
     * @brief Casts VectorND<N,T> to VectorND<Q>
     * @param v [in] VectorND with type T
     * @return VectorND with type Q
     *
     * @relates VectorND
     */
    template< class Q, size_t ND, class T >
    const VectorND< ND, Q > cast (const VectorND< ND, T >& v)
    {
        VectorND< ND, Q > ret;

        for (size_t i = 0; i < v.size (); i++) {
            ret[i] = static_cast< Q > (v[i]);
        }
        return ret;
    }

    template< class T > using Vector6D = VectorND< 6, T >;

#if !defined(SWIG)
    extern template class rw::math::VectorND< 6, double >;
    extern template class rw::math::VectorND< 6, float >;
    extern template class rw::math::VectorND< 5, double >;
    extern template class rw::math::VectorND< 5, float >;
    extern template class rw::math::VectorND< 4, double >;
    extern template class rw::math::VectorND< 4, float >;
    extern template class rw::math::VectorND< 3, double >;
    extern template class rw::math::VectorND< 3, float >;
    extern template class rw::math::VectorND< 2, double >;
    extern template class rw::math::VectorND< 2, float >;
#else
#define VECTORND(num, type) rw::math::VectorND< num, type >;
    SWIG_DECLARE_TEMPLATE (Vector6Dd, VECTORND (6, double));
    SWIG_DECLARE_TEMPLATE (Vector6Df, VECTORND (6, float));
    SWIG_DECLARE_TEMPLATE (Vector5Dd, VECTORND (5, double));
    SWIG_DECLARE_TEMPLATE (Vector5Df, VECTORND (5, float));
    SWIG_DECLARE_TEMPLATE (Vector4Dd, VECTORND (4, double));
    SWIG_DECLARE_TEMPLATE (Vector4Df, VECTORND (4, float));
#undef VECTORND
#endif

    /**@}*/
}}    // namespace rw::math

#endif    // end include guard
