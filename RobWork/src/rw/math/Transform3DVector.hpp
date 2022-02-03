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

#ifndef RW_MATH_TRANSFORM3DVECTOR_HPP
#define RW_MATH_TRANSFORM3DVECTOR_HPP

/**
 * @file Transform3DVector.hpp
 */

#if !defined(SWIG)

#include <rw/core/Ptr.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation3DVector.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

#include <Eigen/Core>

#endif
namespace rw { namespace math {
    /** @addtogroup math */
    /* @{*/

    /**
     * @brief this class is a interpolatable Transform3D, consisting of a Vecor3D and a Quaternion.
     * It is implemented to be very Interconvertable with a Transform3D, and allow operations souch
     * as Transform * scalar and Transform + Transform.
     */
    template< class T = double > class Transform3DVector
    {
      public:
        typedef Eigen::Matrix< T, 7, 1 > type;
        typedef rw::core::Ptr< Transform3DVector< T > > Ptr;

        /**
         * @brief default constructor
         */
        Transform3DVector ():Transform3DVector(Vector3D<T>(),Quaternion<T>()) {}

        /**
         * @brief Constuct a Transformation matrix as a Vector
         * @param vec [in] the vector of the transform
         * @param rot [in] the rotation of the transform
         */
        Transform3DVector (const Vector3D< T >& vec, const Quaternion< T >& rot)
        {
            _t3d[0] = vec[0];
            _t3d[1] = vec[1];
            _t3d[2] = vec[2];
            _t3d[3] = rot[0];
            _t3d[4] = rot[1];
            _t3d[5] = rot[2];
            _t3d[6] = rot[3];
        }

        /**
         * @brief Constuct a Transformation matrix as a Vector
         * @param vec [in] the vector of the transform
         * @param rot [in] the rotation of the transform
         */
        Transform3DVector (const Vector3D< T >& vec, const Rotation3DVector< T >& rot)
        {
            Quaternion< T > rotq (rot);
            _t3d[0] = vec[0];
            _t3d[1] = vec[1];
            _t3d[2] = vec[2];
            _t3d[3] = rotq[0];
            _t3d[4] = rotq[1];
            _t3d[5] = rotq[2];
            _t3d[6] = rotq[3];
        }

        /**
         * @brief Constuct a Transform3DVector from a Transform3D
         * @param t3d [in] Transform3D
         */
        Transform3DVector (const Transform3D< T >& t3d) :
            Transform3DVector (t3d.P (), Quaternion< T > (t3d.R ()))
        {}

        /**
         * @brief Constuct a Transform3DVector from a EigenVector
         * @param vec [in] Transform3D
         */
        Transform3DVector (const type& vec) : _t3d (vec) {}

        /**
         * @brief destructor
         */
        ~Transform3DVector () {}

        // ###################################################
        // #                 Math Operators                  #
        // ###################################################

        // ########## Vector3D Operators

        /**
         * @brief add a Vector3D to the position of the transform
         * @param rhs [in] the right hand side value
         * @return result of devision
         */
        Transform3DVector< T > operator+ (const Vector3D< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i] += rhs[i];
            }
            return ret;
        }

        /**
         * @brief subtract a Vector3D from the position of the transform
         * @param rhs [in] the right hand side value
         * @return result of devision
         */
        Transform3DVector< T > operator- (const Vector3D< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i + 3] -= rhs[i];
            }
            return ret;
        }

        /**
         * @brief Matrix vector multiplication with Vector3D. Same as Transform3D<> * Vector3D<>;
         * @param rhs [in] the right hand side value
         * @return result of multiplication
         */
        Vector3D< T > operator* (const Vector3D< T >& rhs) const
        {
            return this->toTransform3D () * rhs;
        }

        // ########## Quaternion Operators

        /**
         * @brief add a Quaternion to the rotation of the transform
         * @param rhs [in] the right hand side value
         * @return result of devision
         */
        Transform3DVector< T > operator+ (const Quaternion< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i + 3] += rhs[i];
            }
            return ret;
        }

        /**
         * @brief subtract a Quaternion from the rotation of the transform
         * @param rhs [in] the right hand side value
         * @return result of devision
         */
        Transform3DVector< T > operator- (const Quaternion< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i + 3] -= rhs[i];
            }
            return ret;
        }

        /**
         * @brief element wise devide a Quaternion with the Quaternion rotation of the transform
         * @param rhs [in] the right hand side value
         * @return result of devision
         */
        Transform3DVector< T > operator/ (const Quaternion< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i + 3] /= rhs[i];
            }
            return ret;
        }

        /**
         * @brief element wise multiply a Quaternion with the Quaternion rotation of the transform
         * @param rhs [in] the right hand side value
         * @return result of multiplication
         */
        Transform3DVector< T > operator* (const Quaternion< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < rhs.size (); i++) {
                ret[i + 3] *= rhs[i];
            }
            return ret;
        }

        // ########## Transform3DVector Operations

        /**
         * @brief element wise add two Transform3DVectors
         * @param rhs [in] the Transform3D vector to be added with
         * @return the sum of the two objects
         */
        Transform3DVector< T > operator+ (const Transform3DVector< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] += rhs[i];
            }
            return ret;
        }

        /**
         * @brief element wise subtract two Transform3DVectors
         * @param rhs [in] the Transform3D vector to be subtracted with
         * @return the difference of the two objects
         */
        Transform3DVector< T > operator- (const Transform3DVector< T >& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] -= rhs[i];
            }
            return ret;
        }

        // ########## Scalar Operations

        /**
         * @brief Scalar multiplication
         * @param rhs [in] the scalar to multiply with
         * @return product of the multiplication
         */
        Transform3DVector< T > operator* (const T& rhs) const
        {
            return Transform3DVector< T > (this->_t3d * rhs);
        }

#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar multiplication
         * @param lhs [in] the scalar to multiply with
         * @param rhs [in] the Transform3DVector being multiplied with a scalar
         * @return product of the multiplication
         */
        friend Transform3DVector< T > operator* (const T& lhs, const Transform3DVector< T >& rhs)
        {
            return Transform3DVector< T > (lhs * rhs._t3d);
        }
#endif
        /**
         * @brief Scalar addition
         * @param rhs [in] the scalar to add
         * @return the sum
         */
        Transform3DVector< T > operator+ (const T& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] += rhs;
            }
            return ret;
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar addition
         * @param lhs [in] the scalar to subtraction
         * @param rhs [in] the Transform3DVector being subtracted from
         * @return the difference
         */
        friend Transform3DVector< T > operator+ (const T& lhs, const Transform3DVector< T >& rhs)
        {
            Transform3DVector< T > ret = rhs;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] += lhs;
            }
            return ret;
        }
#endif
        /**
         * @brief Scalar subtraction
         * @param rhs [in] the scalar to subtract
         * @return the difference
         */
        Transform3DVector< T > operator- (const T& rhs) const
        {
            Transform3DVector< T > ret = *this;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] -= rhs;
            }
            return ret;
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar subtraction
         * @param lhs [in] the scalar to subtract
         * @param rhs [in] the Transform3DVector being subtracted from
         * @return the difference
         */
        friend Transform3DVector< T > operator- (const T& lhs, const Transform3DVector< T >& rhs)
        {
            Transform3DVector< T > ret = rhs;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] = lhs - rhs[i];
            }
            return ret;
        }
#endif

        /**
         * @brief Scalar devision
         * @param rhs [in] the scalar to devide with
         * @return the result
         */
        Transform3DVector< T > operator/ (const T& rhs) const
        {
            return Transform3DVector< T > (this->_t3d / rhs);
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar devision
         * @param lhs [in] the scalar to devide with
         * @param rhs [in] the Transform3DVector being devided
         * @return the result
         */
        friend Transform3DVector< T > operator/ (const T& lhs, const Transform3DVector< T >& rhs)
        {
            Transform3DVector< T > ret = rhs;
            for (size_t i = 0; i < ret.size (); i++) {
                ret[i] = lhs / rhs[i];
            }
            return ret;
        }
#endif

        // ###################################################
        // #                Acces Operators                  #
        // ###################################################

#if !defined(SWIG)
        /**
         * @brief acces operator
         * @param i [in] index of the Vector
         * @return requested value
         */
        T& operator() (size_t i) { return _t3d[i]; }

        /**
         * @brief acces operator
         * @param i [in] index of the Vector
         * @return requested value
         */
        T operator() (size_t i) const { return _t3d[i]; }

        /**
         * @brief acces operator
         * @param i [in] index of the Vector
         * @return requested value
         */
        T& operator[] (size_t i) { return _t3d[i]; }

        /**
         * @brief acces operator
         * @param i [in] index of the Vector
         * @return requested value
         */
        T operator[] (size_t i) const { return _t3d[i]; }
#else
        ARRAYOPERATOR (T);

#endif
        /**
         * @brief get the size. Index 0-2 Vector, 3-6 Quaternion
         * @return always 7
         */
        size_t size () const { return 7u; }

        /**
         * @brief get the position Vector of the transform
         * @return a copy of the position vector
         */
        Vector3D< T > toVector3D () const { return Vector3D< T > (_t3d[0], _t3d[1], _t3d[2]); }

        /**
         * @brief get the Rotation of the transform
         * @return a copy of the quaternion rotation
         */
        Quaternion< T > toQuaternion () const
        {
            return Quaternion< T > (_t3d[3], _t3d[4], _t3d[5], _t3d[6]);
        }

        /**
         * @brief convert the rotation part to EAA
         * @return toration in EAA form
         */
        EAA< T > toEAA () const { return EAA< T > (this->toQuaternion ().toRotation3D ()); }

        /**
         * @brief Returns the corresponding Trandforma3D matrix
         * @return The transformation matrix
         */
        Transform3D< T > toTransform3D () const
        {
            return Transform3D< T > (this->toVector3D (), this->toQuaternion ());
        }

        /**
         * @brief get the underling eigen type
         * @return reference to eigen vector
         */
        Eigen::Matrix< T, 7, 1 >& e () { return _t3d; }
#if !defined(SWIG)
        friend std::ostream& operator<< (std::ostream& os, const Transform3DVector< T >& t)
        {
            return os << "Transform3DVector(" << t.toVector3D () << ", " << t.toQuaternion ()
                      << ")";
        }
#endif
        // ###################################################
        // #             assignement Operators               #
        // ###################################################

#if !defined(SWIGPYTHON)
        /**
         * @brief copy the transform
         * @param rhs the transform to copy
         * @return a copy of this object
         */
        Transform3DVector< T >& operator= (const Transform3DVector< T >& rhs)
        {
            this->_t3d = rhs._t3d;
            return *this;
        }
#endif
        /**
         * @brief add to the transform
         * @param rhs the transform to be added
         * @return a copy of this object
         */
        Transform3DVector< T >& operator+= (const Transform3DVector< T >& rhs)
        {
            this->_t3d = this->_t3d + rhs._t3d;
            return *this;
        }

        /**
         * @brief subtract from the transform
         * @param rhs the transform to be subtracted
         * @return a copy of this object
         */
        Transform3DVector< T >& operator-= (const Transform3DVector< T >& rhs)
        {
            this->_t3d = this->_t3d - rhs._t3d;
            return *this;
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Override the Roation of the transform
         * @param rhs the new rotation
         * @return a copy of this object
         */
        Transform3DVector< T >& operator= (const Quaternion< T >& rhs)
        {
            *this = Transform3DVector< T > (this->toVector3D (), rhs);
            return *this;
        }
#endif
        /**
         * @brief add to the Roation of the transform
         * @param rhs the rotation to be added
         * @return a copy of this object
         */
        Transform3DVector< T >& operator+= (const Quaternion< T >& rhs)
        {
            *this = *this + rhs;
            return *this;
        }

        /**
         * @brief add to the Roation of the transform
         * @param rhs the rotation to be added
         * @return a copy of this object
         */
        Transform3DVector< T >& operator-= (const Quaternion< T >& rhs)
        {
            *this = *this - rhs;
            return *this;
        }

#if !defined(SWIGPYTHON)
        /**
         * @brief convert and copy the transform
         * @param rhs the transform to copy
         * @return a copy of this object
         */
        Transform3DVector< T >& operator= (const Transform3D< T >& rhs)
        {
            this->_t3d = Transform3DVector< T > (rhs)._t3d;
            return *this;
        }
#endif
#if !defined(SWIGPYTHON)
        /**
         * @brief Override the position of the transform
         * @param rhs the new position
         * @return a copy of this object
         */
        Transform3DVector< T >& operator= (const Vector3D< T >& rhs)
        {
            *this = Transform3DVector< T > (rhs, this->toQuaternion ());
            return *this;
        }
#endif
        /**
         * @brief add to the position of the transform
         * @param rhs the new position
         * @return a copy of this object
         */
        Transform3DVector< T >& operator+= (const Vector3D< T >& rhs)
        {
            *this = *this + rhs;
            return *this;
        }

        /**
         * @brief subtract from the position of the transform
         * @param rhs the new position
         * @return a copy of this object
         */
        Transform3DVector< T >& operator-= (const Vector3D< T >& rhs)
        {
            *this = *this - rhs;
            return *this;
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief copy the transform from an eigen vector
         * @param rhs the transform to copy
         * @return a copy of this object
         */
        Transform3DVector< T >& operator= (const type& rhs)
        {
            this->_t3d = rhs;
            return *this;
        }
#endif
#if !defined(SWIG)
        /**
         * @brief implicit conversion to transform
         */
        operator Transform3D< T > () const { return this->toTransform3D (); }
#endif

      private:
        type _t3d;
    };

#if defined(SWIGPYTHON)

#endif

#if !defined(SWIG)
    extern template class rw::math::Transform3DVector< double >;
    extern template class rw::math::Transform3DVector< float >;

    using Transform3DVectord = Transform3DVector< double >;
    using Transform3DVectorf = Transform3DVector< float >;

#endif
    /**@}*/
}}    // namespace rw::math

#endif    // end include guard
