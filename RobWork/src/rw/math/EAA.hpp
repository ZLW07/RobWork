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

#ifndef RW_MATH_EAA_HPP
#define RW_MATH_EAA_HPP

/**
 * @file EAA.hpp
 */
#if !defined(SWIG)
#include <rw/common/Serializable.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Rotation3DVector.hpp>
#include <rw/math/Vector3D.hpp>

#include <Eigen/Core>
#endif
namespace rw { namespace math {
    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A class for representing an equivalent angle-axis rotation
     *
     * This class defines an equivalent-axis-angle orientation vector also known
     * as an @f$ \thetak @f$ vector or "axis+angle" vector
     *
     * The equivalent-axis-angle vector is the product of a unit vector @f$
     * \hat{\mathbf{k}} @f$ and an angle of rotation around that axis @f$ \theta
     * @f$
     *
     * @note given two EAA vectors @f$ \theta_1\mathbf{\hat{k}}_1 @f$ and @f$
     * \theta_2\mathbf{\hat{k}}_2 @f$ it is generally not possible to subtract
     * or add these vectors, except for the special case when @f$
     * \mathbf{\hat{k}}_1 == \mathbf{\hat{k}}_2 @f$ this is why this class does
     * not have any subtraction or addition operators
     */
    template< class T = double > class EAA : public rw::math::Rotation3DVector< T >
    {
      public:
        /**
         * @brief Extracts Equivalent axis-angle vector from Rotation matrix
         *
         * @param R [in] A 3x3 rotation matrix @f$ \mathbf{R} @f$
         *
         * @f$
         * \theta = arccos(\frac{1}{2}(Trace(\mathbf{R})-1)=arccos(\frac{r_{11}+r_{22}+r_{33}-1}{2})
         * @f$
         *
         * @f$
         * \thetak=log(\mathbf{R})=\frac{\theta}{2 sin \theta}(\mathbf{R}-\mathbf{R}^T) =
         * \frac{\theta}{2 sin \theta}
         * \left[
         * \begin{array}{c}
         * r_{32}-r_{23}\\
         * r_{13}-r_{31}\\
         * r_{21}-r_{12}
         * \end{array}
         * \right]
         * @f$
         *
         * @f$
         * \thetak=
         * \left[
         * \begin{array}{c}
         * 0\\
         * 0\\
         * 0
         * \end{array}
         * \right]
         * @f$ if @f$ \theta = 0 @f$
         *
         * @f$
         * \thetak=\pi
         * \left[
         * \begin{array}{c}
         * \sqrt{(R(0,0)+1.0)/2.0}\\
         * \sqrt{(R(1,1)+1.0)/2.0}\\
         * \sqrt{(R(2,2)+1.0)/2.0}
         * \end{array}
         * \right]
         * @f$ if @f$ \theta = \pi @f$
         *
         */
        explicit EAA (const rw::math::Rotation3D< T >& R);

        /**
         * @brief Constructs an EAA vector initialized to \f$\{0,0,0\}\f$
         */
        EAA () : _eaa (0, 0, 0) {}

        /**
         * @brief Constructs an initialized EAA vector
         * @param axis [in] \f$ \mathbf{\hat{k}} \f$
         * @param angle [in] \f$ \theta \f$
         * @pre norm_2(axis) = 1
         */
        EAA (const rw::math::Vector3D< T >& axis, T angle) : _eaa (axis * angle) {}

        /**
         * @brief Constructs an initialized EAA vector
         * @f$ \thetak =
         * \left[\begin{array}{c}
         *    \theta k_x\\
         *    \theta k_y\\
         *    \theta k_z
         * \end{array}\right]
         * @f$
         * @param thetakx [in] @f$ \theta k_x @f$
         * @param thetaky [in] @f$ \theta k_y @f$
         * @param thetakz [in] @f$ \theta k_z @f$
         */
        EAA (T thetakx, T thetaky, T thetakz) :
            _eaa (rw::math::Vector3D< T > (thetakx, thetaky, thetakz))
        {}

        /**
         * @brief Constructs an EAA vector that will rotate v1 into
         * v2. Where v1 and v2 are normalized and described in the same reference frame.
         * @param v1 [in] normalized vector
         * @param v2 [in] normalized vector
         */
        EAA (const rw::math::Vector3D< T >& v1, const rw::math::Vector3D< T >& v2);

        /**
         * @brief Constructs an initialized EAA vector
         *
         * The angle of the EAA are \f$\|eaa\|\f$ and the axis is \f$\frac{eaa}{\|eaa\|}\f$
         * @param eaa [in] Values to initialize the EAA
         */
        explicit EAA (rw::math::Vector3D< T > eaa) : _eaa (eaa) {}
        
        /**
         * @brief Copy Constructor
         * @param eaa [in] Values to initialize the EAA
         */
        EAA (const rw::math::EAA< T >& eaa) : _eaa (eaa._eaa) {}

        /**
         * @brief Constructs an initialized EAA vector
         *
         * The angle of the EAA are \f$\|eaa\|\f$ and the axis is \f$\frac{eaa}{\|eaa\|}\f$
         * @param eaa [in] Values to initialize the EAA
         */
        template< class R > explicit EAA (const Eigen::MatrixBase< R >& r) : _eaa (r) {}

        //! @brief destructor
        virtual ~EAA () {}

        /**
         * @brief Get the size of the EAA.
         * @return the size (always 3).
         */
        size_t size () const { return 3; }

        // ###################################################
        // #                Acces Operators                  #
        // ###################################################
#if !defined(SWIGJAVA)
        /**
         * @copydoc Rotation3DVector::toRotation3D()
         *
         * @f$
         * \mathbf{R} = e^{[\mathbf{\hat{k}}],\theta}=\mathbf{I}^{3x3}+[\mathbf{\hat{k}}]
         * sin\theta+[{\mathbf{\hat{k}}}]^2(1-cos\theta) = \left[ \begin{array}{ccc}
         *      k_xk_xv\theta + c\theta & k_xk_yv\theta - k_zs\theta & k_xk_zv\theta + k_ys\theta \\
         *      k_xk_yv\theta + k_zs\theta & k_yk_yv\theta + c\theta & k_yk_zv\theta - k_xs\theta\\
         *      k_xk_zv\theta - k_ys\theta & k_yk_zv\theta + k_xs\theta & k_zk_zv\theta + c\theta
         *    \end{array}
         *  \right]
         * @f$
         *
         * where:
         * - @f$ c\theta = cos \theta @f$
         * - @f$ s\theta = sin \theta @f$
         * - @f$ v\theta = 1-cos \theta @f$
         */

#endif 
        virtual const rw::math::Rotation3D< T > toRotation3D () const;

        /**
         * @brief Extracts the angle of rotation @f$ \theta @f$
         * @return @f$ \theta @f$
         */
        T angle () const { return _eaa.norm2 (); }

        /**
         * @brief change the angle of the EAA
         * @param angle [in] the new angle
         * @return this object
         */
        EAA< T >& setAngle (const T& angle)
        {
            (*this) = EAA< T > (this->axis (), angle);
            return (*this);
        }

        /**
         * @brief Extracts the axis of rotation vector @f$ \mathbf{\hat{\mathbf{k}}} @f$
         * @return @f$ \mathbf{\hat{\mathbf{k}}} @f$
         */
        const rw::math::Vector3D< T > axis () const
        {
            T theta = angle ();
            if (theta < 1e-6)
                return rw::math::Vector3D< T > (0, 0, 0);
            else
                return _eaa / theta;
        }

        /**
         * @brief get the underling Vector
         * @return the vector
         */
        rw::math::Vector3D< T >& toVector3D () { return this->_eaa; }

        /**
         * @brief get the underling Vector
         * @return the vector
         */
        rw::math::Vector3D< T > toVector3D () const { return this->_eaa; }

        /**
         * @brief get as eigen vector
         * @return Eigenvector
         */
        Eigen::Matrix< T, 3, 1 >& e () { return this->_eaa.e (); }

        /**
         * @brief get as eigen vector
         * @return Eigenvector
         */
        Eigen::Matrix< T, 3, 1 > e () const { return this->_eaa.e (); }

#if !defined(SWIG)
        /**
         * @brief Returns element of EAA
         * @param i [in] index (@f$ 0 < i < 3 @f$)
         * @return the @f$ i @f$'th element
         */
        const T& operator[] (size_t i) const
        {
            assert (i < 3);
            return _eaa[i];
        }

        /**
         * @brief Returns element of EAA
         * @param i [in] index (@f$ 0 < i < 3 @f$)
         * @return the @f$ i @f$'th element
         */
        T& operator[] (size_t i)
        {
            assert (i < 3);
            return _eaa[i];
        }

        /**
         * @brief Returns element of EAA
         * @param i [in] index (@f$ 0 < i < 3 @f$)
         * @return the @f$ i @f$'th element
         */
        const T& operator() (size_t i) const
        {
            assert (i < 3);
            return _eaa[i];
        }

        /**
         * @brief Returns element of EAA
         * @param i [in] index (@f$ 0 < i < 3 @f$)
         * @return the @f$ i @f$'th element
         */
        T& operator() (size_t i)
        {
            assert (i < 3);
            return _eaa[i];
        }
#else
        ARRAYOPERATOR (T);
#endif
        // ###################################################
        // #                 Math Operators                  #
        // ###################################################

        // ########## Eigen Operations

        /**
         * @brief element wise division.
         * @param rhs [in] the vector being devided with
         * @return the resulting Vector3D
         */
        template< class R > EAA< T > elemDivide (const Eigen::MatrixBase< R >& rhs) const
        {
            EAA< T > ret = *this;
            for (size_t i = 0; i < size (); i++) {
                ret._eaa[i] /= rhs[i];
            }
            return ret;
        }

        /**
         * @brief Elementweise multiplication.
         * @param rhs [in] vector
         * @return the element wise product
         */
        template< class R > EAA< T > elemMultiply (const Eigen::MatrixBase< R >& rhs) const
        {
            EAA< T > ret = *this;
            for (size_t i = 0; i < size (); i++) {
                ret._eaa[i] *= rhs[i];
            }
            return ret;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > EAA< T > operator- (const Eigen::MatrixBase< R >& rhs) const
        {
            return EAA< T > (_eaa - rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend EAA< T > operator- (const Eigen::MatrixBase< R >& lhs, const EAA< T >& rhs)
        {
            return EAA< T > (lhs - rhs.e ());
        }

        /**
         * @brief Vector addition.
         */
        template< class R > EAA< T > operator+ (const Eigen::MatrixBase< R >& rhs) const
        {
            return EAA< T > (_eaa + rhs);
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R >
        friend EAA< T > operator+ (const Eigen::MatrixBase< R >& lhs, const EAA< T >& rhs)
        {
            return EAA< T > (lhs + rhs.e ());
        }

        // ########### EAA Operators

        /**
         * @brief Unary minus.
         * @brief negative version
         */
        EAA< T > operator- () const { return EAA< T > (-_eaa); }

        /**
         * @brief element wise addition
         * @param rhs [in] the EAA to be added
         * @return the sum of the two EAA's
         */
        EAA< T > elemAdd (const EAA< T >& rhs) const { return EAA< T > (this->_eaa + rhs._eaa); }

        /**
         * @brief element wise subtraction
         * @param rhs [in] the EAA to be subtracted
         * @return the difference between the two EAA's
         */
        EAA< T > elemSubtract (const EAA< T >& rhs) const
        {
            return EAA< T > (this->_eaa - rhs._eaa);
        }

        /**
         * @brief element wise devision ( \b this / \b rhs )
         * @param rhs [in] the EAA to be devided with
         * @return the result of division
         */
        EAA< T > elemDivide (const EAA< T >& rhs) const
        {
            return EAA< T > (this->_eaa.elemDivide (rhs._eaa));
        }

        /**
         * @brief element wise multiplication
         * @param rhs [in] the EAA to be multiplyed with
         * @return the result of division
         */
        EAA< T > elemMultiply (const EAA< T >& rhs) const
        {
            return EAA< T > (this->_eaa.elemMultiply (rhs._eaa));
        }

        /**
         * @brief This is rotation multiplcation, and it is multiplication of two EAA's first
         * converted to a Rotation3D
         * @param rhs [in] the eaa to multiply with
         * @return the new rotation
         */
        EAA< T > operator* (const EAA< T >& rhs) const
        {
            return EAA< T > (this->toRotation3D () * rhs.toRotation3D ());
        }

        // ########### Rotation3D Operators
#if !defined(SWIG)
        /**
         * @brief Calculates \f$ \robabx{a}{c}{\thetak} =
         * \robabx{a}{b}{\mathbf{R}} \robabx{b}{c}{\mathbf{\thetak}} \f$
         *
         * @param aRb [in] \f$ \robabx{a}{b}{\mathbf{R}} \f$
         * @param bTKc [in] \f$ \robabx{b}{c}{\thetak} \f$
         * @return \f$ \robabx{a}{c}{\thetak} \f$
         */
        friend EAA< T > operator* (const rw::math::Rotation3D< T >& aRb, const EAA< T >& bTKc)
        {
            return EAA (aRb * bTKc._eaa);
        }
#endif

        /**
         * @brief matrix multiplication converting EAA to rotation
         * @param rhs [in] the roation matrix to multiply with
         * @return EAA ( this->toRotation3D() * rhs)
         */
        template< class R > EAA< T > operator* (const rw::math::Rotation3D< R >& rhs)
        {
            return EAA (this->toRotation3D () * rhs);
        }

        // ########### Scalar operators

        /**
         * @brief scalar multiplication
         * @param rhs [in] the scalar to multiply with
         * @return the product
         */
        EAA< T > elemMultiply (const T& rhs) const { return EAA< T > (this->_eaa * rhs); }

        /**
         * @brief scalar devision
         * @param rhs [in] the scalar to devide with
         * @return the resulting EAA
         */
        EAA< T > elemDivide (const T& rhs) const { return EAA< T > (this->_eaa / rhs); }

        /**
         * @brief Scalar subtraction.
         */
        EAA< T > elemSubtract (const T rhs) const { return EAA< T > (_eaa.elemSubtract (rhs)); }

        /**
         * @brief Scalar addition.
         */
        EAA< T > elemAdd (const T rhs) const { return EAA< T > (_eaa.elemAdd (rhs)); }

        /**
         * @brief scale the angle, keeping the axis the same
         * @param scale [in] how much the angle should change
         * @return a new EAA with the scaled angle
         */
        EAA< T > scaleAngle (const T& scale)
        {
            return EAA< T > (this->axis (), this->angle () * scale);
        }

        // ############ Vector3D Operators

        /**
         * @brief element wise multiplication.
         * @param rhs [in] the vector being devided with
         * @return the resulting EAA
         */
        EAA< T > operator+ (const rw::math::Vector3D< T >& rhs) const
        {
            return EAA< T > (this->_eaa + rhs);
        }

#if !defined(SWIG)
        /**
         * @brief Vector addition
         * @param lhs [in] left side value
         * @param rhs [in] right side value
         * @return the resulting EAA
         */
        friend EAA< T > operator+ (const rw::math::Vector3D< T >& lhs, const EAA< T >& rhs)
        {
            return EAA< T > (lhs + rhs._eaa);
        }
#endif

        /**
         * @brief Vector addition
         * @param rhs [in] the vector being added
         * @return the resulting EAA
         */
        EAA< T > operator- (const rw::math::Vector3D< T >& rhs) const
        {
            return EAA< T > (this->_eaa - rhs);
        }

#if !defined(SWIG)
        /**
         * @brief Vector addition
         * @param lhs [in] left side value
         * @param rhs [in] right side value
         * @return the resulting EAA
         */
        friend EAA< T > operator- (const rw::math::Vector3D< T >& lhs, const EAA< T >& rhs)
        {
            return EAA< T > (lhs - rhs._eaa);
        }
#endif
        /**
         * @brief element wise devision ( \b this / \b rhs )
         * @param rhs [in] the Vector to be devided with
         * @return the result of division
         */
        EAA< T > elemDivide (const rw::math::Vector3D< T >& rhs) const
        {
            return EAA< T > (this->_eaa.elemDivide (rhs));
        }

        /**
         * @brief element wise multiplication
         * @param rhs [in] the Vector to be multiplyed with
         * @return the result of division
         */
        EAA< T > elemMultiply (const rw::math::Vector3D< T >& rhs) const
        {
            return EAA< T > (this->_eaa.elemMultiply (rhs));
        }

        // ############ Ostream operators

#if !defined(SWIG)
        /**
         * @brief Ouputs EAA to stream
         * @param os [in/out] stream to use
         * @param eaa [in] equivalent axis-angle
         * @return the resulting stream
         */
        friend std::ostream& operator<< (std::ostream& os, const EAA< T >& eaa)
        {
            return os << " EAA( " << eaa (0) << ", " << eaa (1) << ", " << eaa (2) << ")";
        }
#else
        TOSTRING (rw::math::EAA< T >);
#endif
        // ############ Math Operations

        /**
         * @brief Calculates the cross product and returns the result
         * @param v [in] a Vector3D
         * @return the resulting 3D vector
         */
        rw::math::Vector3D< T > cross (const rw::math::Vector3D< T >& v) const
        {
            return rw::math::cross (this->_eaa, v);
        }

        /**
         * @brief Calculates the cross product and returns the result
         * @param eaa [in] a EAA
         * @return the resulting 3D vector
         */
        EAA< T > cross (const EAA< T >& eaa) const
        {
            return EAA< T > (rw::math::cross (this->_eaa, eaa._eaa));
        }

        /**
         * @brief Calculates the dot product and returns the result
         * @param v [in] a Vector3D
         * @return the resulting scalar
         */
        T dot (const rw::math::Vector3D< T >& v) { return rw::math::dot (this->_eaa, v); }

        /**
         * @brief Calculates the cross product and returns the result
         * @param eaa [in] a EAA
         * @return the resulting 3D vector
         */
        T dot (const EAA< T >& eaa) { return rw::math::dot (this->_eaa, eaa._eaa); }

        /**
         * @brief Returns the Euclidean norm (2-norm) of the vector
         * @return the norm
         */
        T norm2 () const { return _eaa.norm2 (); }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the vector
         * @return the norm
         */
        T norm1 () const { return _eaa.norm1 (); }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the vector
         * @return the norm
         */
        T normInf () const { return _eaa.normInf (); }

        // ###################################################
        // #             assignement Operators               #
        // ###################################################

        /**
         * @brief copy operator
         * @param rhs [in] the EAA to be copied
         * @return reference to this EAA
         */
        EAA< T >& operator= (const EAA< T >& rhs)
        {
            this->_eaa = rhs._eaa;
            return *this;
        }

        /**
         * @brief assign vector to EAA
         * @param rhs [in] the vector to asign
         * @return reference to this EAA
         */
        EAA< T >& operator= (const rw::math::Vector3D< T >& rhs)
        {
            this->_eaa = rhs;
            return *this;
        }

        /**
         * @brief addition operator
         * @param rhs [in] the right hand side of the operation
         * @return reference to this EAA
         */
        EAA< T >& operator+= (const rw::math::Vector3D< T >& rhs)
        {
            this->_eaa += rhs;
            return *this;
        }

        /**
         * @brief subtraction operator
         * @param rhs [in] the right hand side of the operation
         * @return reference to this EAA
         */
        EAA< T >& operator-= (const rw::math::Vector3D< T >& rhs)
        {
            this->_eaa -= rhs;
            return *this;
        }
#if !defined(SWIG)
        /**
         * @brief Implicit converter to Vector3D
         */
        operator rw::math::Vector3D< T > () const { return _eaa; }

        /**
         * @brief Implicit converter to Vector3D
         */
        operator rw::math::Vector3D< T > & () { return _eaa; }
#endif
        /**
         * @brief copy a vector from eigen type
         * @param r [in] an Eigen Vector
         */
        template< class R > EAA< T >& operator= (const Eigen::MatrixBase< R >& r)
        {
            _eaa = r;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        template< class R > EAA< T >& operator+= (const Eigen::MatrixBase< R >& r)
        {
            _eaa += r;
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        template< class R > EAA< T >& operator-= (const Eigen::MatrixBase< R >& r)
        {
            _eaa -= r;
            return *this;
        }
#if !defined(SWIG)
        /**
         * @brief implicit conversion to EigenVector
         */
        operator Eigen::Matrix< T, 3, 1 > () const { return this->e (); }

        /**
         * @brief implicit conversion to EigenVector
         */
        operator Eigen::Matrix< T, 3, 1 > & () { return this->e (); }
#endif

        /**
         * @brief copy operator
         * @param rhs [in] the Rotation3D to be copied
         * @return reference to this EAA
         */
        EAA< T >& operator= (const rw::math::Rotation3D< T >& rhs)
        {
            return (*this) = EAA< T > (rhs);
        }

        // ###################################################
        // #                    Comparetors                  #
        // ###################################################

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        bool operator== (const rw::math::Vector3D< T >& rhs) const { return _eaa == rhs; }

#if !defined(SWIG)
        /**
         * @brief Compare with \b rhs for equality.
         * @param lhs [in] first Vector
         * @param rhs [in] second vector.
         * @return True if a equals b, false otherwise.
         */
        friend bool operator== (const rw::math::Vector3D< T >& lhs, const EAA< T >& rhs)
        {
            return lhs == rhs._eaa;
        }
#endif

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param rhs [in] other vector.
         *  @return True if a and b are different, false otherwise.
         */
        bool operator!= (const rw::math::Vector3D< T >& rhs) const { return _eaa != rhs; }

#if !defined(SWIG)
        /**
         *  @brief Compare with \b rhs for inequality.
         * @param lhs [in] first Vector
         * @param rhs [in] second vector.
         *  @return True if a and b are different, false otherwise.
         */
        friend bool operator!= (const rw::math::Vector3D< T >& lhs, const EAA< T >& rhs)
        {
            return lhs != rhs._eaa;
        }
#endif

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        bool operator== (const EAA< T >& rhs) const { return _eaa == rhs._eaa; }

        /**
         *  @brief Compare with \b rhs for inequality.
         *  @param rhs [in] other vector.
         *  @return True if a and b are different, false otherwise.
         */
        bool operator!= (const EAA< T >& rhs) const { return _eaa != rhs._eaa; }

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        template< class R > bool operator== (const Eigen::MatrixBase< R >& rhs) const
        {
            return this->_eaa == rhs;
        }

        /**
         * @brief Compare with \b rhs for equality.
         * @param rhs [in] other vector.
         * @return True if a equals b, false otherwise.
         */
        template< class R >
        friend bool operator== (const Eigen::MatrixBase< R >& lhs, const EAA< T >& rhs)
        {
            return lhs == rhs._eaa;
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
        friend bool operator!= (const Eigen::MatrixBase< R >& lhs, const EAA< T >& rhs)
        {
            return !(lhs == rhs);
        }

      private:
        rw::math::Vector3D< T > _eaa;
    };

    template< class T >
    rw::math::Vector3D< T > cross (const rw::math::Vector3D< T >& v1, const EAA< T >& v2)
    {
        return rw::math::cross (v1, v2.axis () * v2.angle ());
    }

    /**
     * @brief Casts EAA<T> to EAA<Q>
     * @param eaa [in] EAA with type T
     * @return EAA with type Q
     */
    template< class Q, class T > const EAA< Q > cast (const EAA< T >& eaa)
    {
        return EAA< Q > (
            static_cast< Q > (eaa (0)), static_cast< Q > (eaa (1)), static_cast< Q > (eaa (2)));
    }

    using EAAd = EAA< double >;
    using EAAf = EAA< float >;

    /*@}*/

}}    // namespace rw::math

#if !defined(SWIG)
extern template class rw::math::EAA< double >;
extern template class rw::math::EAA< float >;
#else
SWIG_DECLARE_TEMPLATE (EAAd, rw::math::EAA< double >);
SWIG_DECLARE_TEMPLATE (EAAf, rw::math::EAA< float >);
#endif

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::EAA
         */
        template<>
        void write (const rw::math::EAA< double >& sobject, rw::common::OutputArchive& oarchive,
                    const std::string& id);

        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::EAA
         */
        template<>
        void write (const rw::math::EAA< float >& sobject, rw::common::OutputArchive& oarchive,
                    const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::EAA
         */
        template<>
        void read (rw::math::EAA< double >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::EAA
         */
        template<>
        void read (rw::math::EAA< float >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

#endif    // end include guard
