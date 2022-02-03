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

#ifndef RW_MATH_QUATERNION_HPP
#define RW_MATH_QUATERNION_HPP

/**
 * @file Quaternion.hpp
 */

#if !defined(SWIG)
#include <rw/common/Serializable.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Rotation3DVector.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ostream>
#endif

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A Quaternion @f$ \mathbf{q}\in \mathbb{R}^4 @f$ a complex
     * number used to describe rotations in 3-dimensional space.
     * @f$ q_w+{\bf i}\ q_x+ {\bf j} q_y+ {\bf k}\ q_z @f$
     *
     * Quaternions can be added and multiplied in a similar way as usual
     * algebraic numbers. Though there are differences. Quaternion
     * multiplication is not commutative which means
     * \f$ Q\cdot P \neq P\cdot Q \f$
     */
    template< class T = double > class Quaternion : public rw::math::Rotation3DVector< T >
    {
      private:
        typedef Eigen::Quaternion< T > EigenQuaternion;


      public:

        //! Value type.
        typedef T value_type;

        /**
         * @brief constuct Quaterinion of {0,0,0,1}
         */
        Quaternion () : _q (0, 0, 0, 1) {}

        /**
         * @brief Creates a Quaternion
         * @param qx [in] @f$ q_x @f$
         * @param qy [in] @f$ q_y @f$
         * @param qz [in] @f$ q_z @f$
         * @param qw  [in] @f$ q_w @f$
         */
        Quaternion (T qx, T qy, T qz, T qw) : _q (qw, qx, qy, qz) {}

        /**
         * @brief Creates a Quaternion from another Quaternion
         * @param quat [in] Quaternion
         */
        Quaternion (const Quaternion< T >& quat) : _q (quat._q) {}

        /**
         * @brief Creates a Quaternion from another Rotation3DVector type
         * @param rot [in] The Rotation3DVector type
         */
        Quaternion (const rw::math::Rotation3DVector< T >& rot)
        {
            setRotation (rot.toRotation3D ());
        }

        /**
         * @brief Extracts a Quaternion from Rotation matrix using
         * setRotation(const Rotation3D<R>& rot)
         * @param rot [in] A 3x3 rotation matrix @f$ \mathbf{rot} @f$
         */
        Quaternion (const rw::math::Rotation3D< T >& rot) { setRotation (rot); }

        /**
         * @brief Creates a Quaternion from a Eigen quaternion
         * @param r [in] a boost quaternion
         */
        Quaternion (const Eigen::Quaternion< T >& r) : _q (r) {}

        /**
         * @brief Creates a Quaternion from vector_expression
         *
         * @param r [in] an Eigen Vector
         */
        template< class R >
        explicit Quaternion (const Eigen::MatrixBase< R >& r) :
            _q (r.row (3) (0), r.row (0) (0), r.row (1) (0), r.row (2) (0))
        {}

        // ###################################################
        // #                Acces Operators                  #
        // ###################################################

        /**
         * @brief get method for the x component
         * @return the x component of the quaternion
         */
        inline T getQx () const { return _q.x (); }

        /**
         * @brief get method for the y component
         * @return the y component of the quaternion
         */
        inline T getQy () const { return _q.y (); }

        /**
         * @brief get method for the z component
         * @return the z component of the quaternion
         */
        inline T getQz () const { return _q.z (); }

        /**
         * @brief get method for the w component
         * @return the w component of the quaternion
         */
        inline T getQw () const { return _q.w (); }

#if !defined(SWIG)
        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return const reference to element
         */
        inline T operator() (size_t i) const
        {
            switch (i) {
                case 0: return _q.x ();
                case 1: return _q.y ();
                case 2: return _q.z ();
                case 3: return _q.w ();
                default: assert (0); return _q.x ();
            }
        }

        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return reference to element
         */
        inline T& operator() (size_t i)
        {
            switch (i) {
                case 0: return _q.x ();
                case 1: return _q.y ();
                case 2: return _q.z ();
                case 3: return _q.w ();
                default: assert (0); return _q.x ();
            }
        }

        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return reference to element
         */
        inline T& operator[] (size_t i)
        {
            switch (i) {
                case 0: return _q.x ();
                case 1: return _q.y ();
                case 2: return _q.z ();
                case 3: return _q.w ();
                default: assert (0); return _q.x ();
            }
        }

        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return reference to element
         */
        inline T operator[] (size_t i) const
        {
            switch (i) {
                case 0: return _q.x ();
                case 1: return _q.y ();
                case 2: return _q.z ();
                case 3: return _q.w ();
                default: assert (0); return _q.x ();
            }
        }
#else
        ARRAYOPERATOR (T);
#endif
#if !defined(SWIGJAVA)

        /**
         * @brief Calculates the @f$ 3\times 3 @f$ Rotation matrix
         *
         * @return A 3x3 rotation matrix @f$ \mathbf{rot} @f$
         * @f$
         * \mathbf{rot} =
         *  \left[
         *   \begin{array}{ccc}
         *      1-2(q_y^2-q_z^2) & 2(q_x\ q_y+q_z\ q_w)& 2(q_x\ q_z-q_y\ q_w) \\
         *      2(q_x\ q_y-q_z\ q_w) & 1-2(q_x^2-q_z^2) & 2(q_y\ q_z+q_x\ q_w)\\
         *      2(q_x\ q_z+q_y\ q_w) & 2(q_y\ q_z-q_x\ q_z) & 1-2(q_x^2-q_y^2)
         *    \end{array}
         *  \right]
         * @f$
         *
         */
#endif
        inline const rw::math::Rotation3D< T > toRotation3D () const
        {
            const T qx = _q.x ();
            const T qy = _q.y ();
            const T qz = _q.z ();
            const T qw = _q.w ();

            return rw::math::Rotation3D< T > (1 - 2 * qy * qy - 2 * qz * qz,
                                              2 * (qx * qy - qz * qw),
                                              2 * (qx * qz + qy * qw),
                                              2 * (qx * qy + qz * qw),
                                              1 - 2 * qx * qx - 2 * qz * qz,
                                              2 * (qy * qz - qx * qw),
                                              2 * (qx * qz - qy * qw),
                                              2 * (qy * qz + qx * qw),
                                              1 - 2 * qx * qx - 2 * qy * qy);
        }

        /** @brief Converts a Rotation3D to a Quaternion and saves the Quaternion
         * in this.
         *
         * @param rot [in] A 3x3 rotation matrix @f$ \mathbf{R} @f$
         *
         * @f$
         * \begin{array}{c}
         * q_x\\ q_y\\ q_z\\ q_w
         * \end{array}
         * =
         *  \left[
         *   \begin{array}{c}
         *      \\
         *      \\
         *
         *    \end{array}
         *  \right]
         * @f$
         *
         * The conversion method is proposed by Henrik Gordon Petersen. The switching between
         * different cases occur well before numerical instabilities, hence the solution should be
         * more robust, than many of the methods proposed elsewhere.
         *
         */
        template< class R > void setRotation (const rw::math::Rotation3D< R >& rot)
        {
            // The method
            const T min  = (T) (-0.9);
            const T min1 = (T) (min / 3.0);

            const T tr = rot (0, 0) + rot (1, 1) + rot (2, 2);

            if (tr > min) {
                const T s = static_cast< T > (0.5) / static_cast< T > (sqrt (tr + 1.0));
                _q.w ()   = static_cast< T > (0.25) / s;
                _q.x ()   = static_cast< T > (rot (2, 1) - rot (1, 2)) * s;
                _q.y ()   = static_cast< T > (rot (0, 2) - rot (2, 0)) * s;
                _q.z ()   = static_cast< T > (rot (1, 0) - rot (0, 1)) * s;
            }
            else {
                if (rot (0, 0) > min1) {
                    const T sa =
                        static_cast< T > (sqrt (rot (0, 0) - rot (1, 1) - rot (2, 2) + 1.0));
                    _q.x ()   = static_cast< T > (0.5) * sa;
                    const T s = static_cast< T > (0.25) / _q.x ();
                    _q.y ()   = static_cast< T > (rot (0, 1) + rot (1, 0)) * s;
                    _q.z ()   = static_cast< T > (rot (0, 2) + rot (2, 0)) * s;
                    _q.w ()   = static_cast< T > (rot (2, 1) - rot (1, 2)) * s;
                }
                else if (rot (1, 1) > min1) {
                    const T sb = static_cast< T > (sqrt (rot (1, 1) - rot (2, 2) - rot (0, 0) + 1));
                    _q.y ()    = static_cast< T > (0.5) * sb;

                    const T s = static_cast< T > (0.25) / _q.y ();
                    _q.x ()   = static_cast< T > (rot (0, 1) + rot (1, 0)) * s;
                    _q.z ()   = static_cast< T > (rot (1, 2) + rot (2, 1)) * s;
                    _q.w ()   = static_cast< T > (rot (0, 2) - rot (2, 0)) * s;
                }
                else {
                    const T sc = static_cast< T > (sqrt (rot (2, 2) - rot (0, 0) - rot (1, 1) + 1));
                    _q.z ()    = static_cast< T > (0.5) * sc;

                    const T s = static_cast< T > (0.25) / _q.z ();
                    _q.x ()   = static_cast< T > (rot (0, 2) + rot (2, 0)) * s;
                    _q.y ()   = static_cast< T > (rot (1, 2) + rot (2, 1)) * s;
                    _q.w ()   = static_cast< T > (rot (1, 0) - rot (0, 1)) * s;
                }
            }
        }

        /**
         * @brief The dimension of the quaternion (i.e. 4).
         * This method is provided to help support generic algorithms using
         * size() and operator[].
         */
        size_t size () const { return 4; }

        /**
         * @brief Convert to an Eigen Quaternion.
         * @return Eigen Quaternion representation.
         */
        Eigen::Quaternion< T >& e () { return _q; }

        //! @copydoc e()
        const Eigen::Quaternion< T >& e () const { return _q; }

        /**
         * @brief convert to Eigen Vector
         * @return eigen Vector of quaternion
         */
        Eigen::Matrix< T, 4, 1 > toEigenVector () const
        {
            return Eigen::Matrix< T, 4, 1 > (_q.x (), _q.y (), _q.z (), _q.w ());
        }

        // ###################################################
        // #                 Math Operators                  #
        // ###################################################

        // ############ Quaternion Operations

        /**
         * @brief Unary minus.
         */
        Quaternion< T > operator- () const
        {
            return Quaternion (-_q.x (), -_q.y (), -_q.z (), -_q.w ());
        }

        /**
         * @brief Unary plus.
         */
        Quaternion< T > operator+ () const { return Quaternion (*this); }

        /**
         * @brief Subtraction.
         */
        inline const Quaternion< T > operator- (const Quaternion< T >& v)
        {
            return Quaternion< T > (
                (*this) (0) - v (0), (*this) (1) - v (1), (*this) (2) - v (2), (*this) (3) - v (3));
        }

        /**
         * @brief Multiply-from operator
         */
        inline Quaternion< T > operator* (const Quaternion< T >& r) const
        {
            Quaternion q = Quaternion (_q * r.e ());
            return q;
        }

        /**
           @brief Addition of two quaternions
         */
        inline const Quaternion< T > operator+ (const Quaternion< T >& v) const
        {
            return Quaternion< T > (
                (*this) (0) + v (0), (*this) (1) + v (1), (*this) (2) + v (2), (*this) (3) + v (3));
        }

        // ############ Scalar Operations

        /**
         * @brief Scalar multiplication.
         */
        inline const Quaternion< T > operator* (T s) const
        {
            return Quaternion< T > (_q.x () * s, _q.y () * s, _q.z () * s, _q.w () * s);
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar multiplication.
         */
        inline friend const Quaternion< T > operator* (T s, const Quaternion< T >& v)
        {
            return v * s;
        }
#endif
        /**
         * @brief element whise division
         * @param lhs [in] the scalar to devide with
         * @return the result of elementwise devision
         */
        Quaternion< T > elemDivide (const T& lhs) const;

        // ############ Math Operations

        /**
         * @brief get length of quaternion
         * @f$ \sqrt{q_x^2+q_y^2+q_z^2+q_w^2} @f$
         * @return the length og this quaternion
         */
        inline T getLength () const { return _q.norm (); }

        /**
         * @brief get squared length of quaternion
         * @f$ q_x^2+q_y^2+q_z^2+q_w^2 @f$
         * @return the length og this quaternion
         */
        inline T getLengthSquared () const { return _q.squaredNorm (); }

        /**
         * @brief normalizes this quaternion so that
         * @f$ normalze(Q)=\frac{Q}{\sqrt{q_x^2+q_y^2+q_z^2+q_w^2}} @f$
         */
        inline void normalize () { _q.normalize (); };

        /**
         * @brief Calculates a slerp interpolation between \b this and \b v.
         *
         * The slerp interpolation ensures a constant velocity across the interpolation.
         * For \f$ t=0\f$ the result is \b this and for \f$ t=1\f$ it is \b v.
         *
         * @note Algorithm and implementation is thanks to euclideanspace.com
         */
        inline const Quaternion< T > slerp (const Quaternion< T >& v, const T t) const
        {
            return Quaternion (_q.slerp (t, v.e ()));
        }

        /*
         * @brief this will return the exponential of this quaternion \f$ e^Quaternion \f$
         * @return the exponential of this quaternion
         */
        Quaternion< T > exp () const;

        /**
         * @brief Calculate the inverse Quaternion
         * @return the inverse quaternion
         */
        Quaternion< T > inverse () const;

        /**
         * @brief calculates the natural logerithm of this quaternion
         * @return natural logetihm
         */
        Quaternion< T > ln () const;

        /**
         * @brief calculates the quaternion lifted to the power of \b power
         * @param power [in] the power the quaternion is lifted to
         * @return \f$ Quaternion^power \f$
         */
        Quaternion< T > pow (double power) const;

        // ###################################################
        // #             assignement Operators               #
        // ###################################################

        /**
         * @brief copy a boost quaternion to this Quaternion
         * @param r [in] - boost quaternion
         */
        inline void operator= (const Eigen::Quaternion< T >& r) { _q = r; }

        /**
           @brief Scalar multiplication.
         */
        inline const Quaternion< T > operator*= (T s)
        {
            _q.x () *= s;
            _q.y () *= s;
            _q.z () *= s;
            _q.w () *= s;
            return *this;
        }

        /**
         * @brief Multiply with operator
         */
        inline const Quaternion< T > operator*= (const Quaternion< T >& r)
        {
            *this = (*this) * r;
            return *this;
        }

        /**
         *@brief Add-to operator
         */
        inline const Quaternion< T > operator+= (const Quaternion< T >& r)
        {
            _q.x () += r (0);
            _q.y () += r (1);
            _q.z () += r (2);
            _q.w () += r (3);
            return *this;
        }

        /**
         * @brief Subtract-from operator
         */
        inline const Quaternion< T > operator-= (const Quaternion< T >& r)
        {
            _q.x () -= r (0);
            _q.y () -= r (1);
            _q.z () -= r (2);
            _q.w () -= r (3);
            return *this;
        }

        /**
         * @brief copyfrom rotaion matrix, same as setRotation.
         * @param rhs [in] the rotation that will be copyed
         */
        Quaternion< T >& operator= (const rw::math::Rotation3D<>& rhs)
        {
            this->setRotation (rhs);
            return (*this);
        }

        // ###################################################
        // #              Comparison Operators               #
        // ###################################################

        /**
         * @brief Comparison (equals) operator
         */
        inline bool operator== (const Quaternion< T >& r) const
        {
            return (*this) (0) == r (0) && (*this) (1) == r (1) && (*this) (2) == r (2) &&
                   (*this) (3) == r (3);
        }

        /**
         * @brief Comparison (not equals) operator
         */
        inline bool operator!= (const Quaternion< T >& r) const { return !((*this) == r); }

#if defined(SWIG)
        TOSTRING ();
#endif
      private:
        Eigen::Quaternion< T > _q;
    };

    /**
       @brief Streaming operator.

       @relates Quaternion
    */
    template< class T > std::ostream& operator<< (std::ostream& out, const Quaternion< T >& v)
    {
        return out << "Quaternion {" << v (0) << ", " << v (1) << ", " << v (2) << ", " << v (3)
                   << "}";
    }

    /**
     * @brief calculates the natural logerithm of this quaternion
     * @param q [in] the quaternion being operated on
     * @return natural logetihm
     */
    template< class T > Quaternion< T > ln (const Quaternion< T >& q) { return q.ln (); }

    /**
     * @brief this will return the exponential of this quaternion \f$ e^Quaternion \f$
     * @param q [in] the quaternion being operated on
     * @return the exponential of this quaternion
     */
    template< class T > Quaternion< T > exp (const Quaternion< T >& q) { return q.exp (); }

    /**
     * @brief Calculate the inverse Quaternion
     * @param q [in] the quaternion being operated on
     * @return the inverse quaternion
     */
    template< class T > Quaternion< T > inverse (const Quaternion< T >& q) { return q.inverse (); }

    /**
     * @brief calculates the quaternion lifted to the power of \b power
     * @param q [in] the quaternion being operated on
     * @param power [in] the power the quaternion is lifted to
     * @return \f$ Quaternion^power \f$
     */
    template< class T > Quaternion< T > pow (const Quaternion< T >& q, double power)
    {
        return q.pow (power);
    }

    /**
     * @brief Casts Quaternion<T> to Quaternion<Q>
     * @param quaternion [in] Quarternion with type T
     * @return Quaternion with type Q
     */
    template< class Q, class T >
    inline const Quaternion< Q > cast (const Quaternion< T >& quaternion)
    {
        return Quaternion< Q > (static_cast< Q > (quaternion (0)),
                                static_cast< Q > (quaternion (1)),
                                static_cast< Q > (quaternion (2)),
                                static_cast< Q > (quaternion (3)));
    }
#if !defined(SWIG)
    extern template class rw::math::Quaternion< double >;
    extern template class rw::math::Quaternion< float >;
#else

#if SWIG_VERSION < 0x040000
    SWIG_DECLARE_TEMPLATE (Quaternion_d, rw::math::Quaternion< double >);
    ADD_DEFINITION (Quaternion_d, Quaternion)
#else
    SWIG_DECLARE_TEMPLATE (Quaternion, rw::math::Quaternion< double >);
#endif
    SWIG_DECLARE_TEMPLATE (Quaternion_f, rw::math::Quaternion< float >);
#endif
    using Quaterniond = Quaternion< double >;
    using Quaternionf = Quaternion< float >;

    /*@}*/
}}    // namespace rw::math

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Quaternion
         */
        template<>
        void write (const rw::math::Quaternion< double >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Quaternion
         */
        template<>
        void write (const rw::math::Quaternion< float >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Quaternion
         */
        template<>
        void read (rw::math::Quaternion< double >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Quaternion
         */
        template<>
        void read (rw::math::Quaternion< float >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

#endif    // end include guard
