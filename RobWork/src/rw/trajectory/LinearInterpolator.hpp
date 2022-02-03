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

#ifndef RW_TRAJECTORY_LINEARINTERPOLATOR_HPP
#define RW_TRAJECTORY_LINEARINTERPOLATOR_HPP

/**
 * @file LinearInterpolator.hpp
 */
#if !defined(SWIG)
#include "Interpolator.hpp"

#include <rw/core/macros.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Transform3D.hpp>
#endif 
namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Make a linear interpolation between to position
     *
     * Given a start \f$ \mathbf{s}\f$, end \f$\mathbf{e}\f$ and duration \f$ d\f$
     * the interpolation is implemented as \f$\mathbf{x}(t)=\mathbf{s} +
     * (\mathbf{e}-\mathbf{s})*t/d\f$.
     *
     * The template argument given needs to support addition with the "+" operator
     * and scaling with a double using the "*" operator.
     *
     * For use with a rw::math::Transform3D see the template specialization
     */
    template< class T > class LinearInterpolator : public Interpolator< T >
    {
      public:
        //! @brief smart pointer type to instance of class
        typedef typename rw::core::Ptr< LinearInterpolator > Ptr;

        //! @brief smart pointer type const instance of class
        typedef typename rw::core::Ptr< const LinearInterpolator > CPtr;

        /**
         * @brief Construct LinearInterpolator starting a \b start and finishing in \b end
         * and taking \b duration time.
         *
         * If \b duration <= 0 an exception is thrown
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        LinearInterpolator (const T& start, const T& end, double duration) :
            _a (end-start), _b (start), _duration (duration)
        {
            if (_duration <= 0)
                RW_THROW ("Duration of an interpolator need to be greater than 0");

            _a = _a / _duration;
        }

        /**
         * @cond
         * @copydoc Interpolator::x()
         * @endcond
         */
        T x (double t) const
        {
            return _a *t + _b;
        }

        /**
         * @cond
         * @copydoc Interpolator::dx()
         * @endcond
         */
        T dx (double t) const { return _a; };

        /**
         * @cond
         * @copydoc Interpolator::ddx()
         * @endcond
         */
        T ddx (double t) const { return _b*0; }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        T getStart () const { return _b; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        T getEnd () const { return _a *_duration + _b; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration () const { return _duration; }

      private:
        T _a;
        T _b;
        double _duration;
    };

    /**
     * @brief Forward declaration for parabolic blend to make the
     * LinearInterpolator<rw::math::Transform3D<T> > a friend
     */
    template< class T > class ParabolicBlend;

    /**
     * @brief Implements LinearInterpolator for rw::math::Rotation3D<T>
     *
     * The interpolation of rotation is made using a Quaternion slerp interpolation.
     * See rw::math::Quaternion::slerp for further information.
     *
     */
    template< class T >
    class LinearInterpolator< rw::math::Rotation3D< T > >
        : public Interpolator< rw::math::Rotation3D< T > >
    {
      public:
        //! @brief smart pointer type to this class
        typedef typename rw::core::Ptr< LinearInterpolator< rw::math::Rotation3D< T > > > Ptr;

        //! @brief smart pointer type const instance of class
        typedef typename rw::core::Ptr< const LinearInterpolator< rw::math::Rotation3D< T > > >
            CPtr;

        /**
         * @brief Construct LinearInterpolator starting a \b start and finishing in \b end
         * and taking \b duration time.
         *
         * If \b duration <= 0 an exception is thrown
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        LinearInterpolator (const rw::math::Rotation3D< T >& start,
                            const rw::math::Rotation3D< T >& end, double duration) :
            _start (start),
            _end (end), _quarStart (start), _quarEnd (end), _duration (duration)
        {
            if (_duration <= 0)
                RW_THROW ("Duration of a interpolator need to have a positive value");

            T d1 = 0;
            T d2 = 0;
            for (size_t i = 0; i < 4; i++) {
                d1 += rw::math::Math::sqr (_quarStart (i) - _quarEnd (i));
                d2 += rw::math::Math::sqr (_quarStart (i) + _quarEnd (i));
            }
            if (d1 > d2)
                _quarEnd = _quarEnd * (-1);
        }

        /**@cond
         * @copydoc Interpolator::x()
         * @endcond
         */
        rw::math::Rotation3D< T > x (double t) const
        {
            return _quarStart.slerp (_quarEnd, t / _duration).toRotation3D ();
        }

        /**
         * @cond
         * @copydoc Interpolator::dx()
         * @endcond
         */
        rw::math::Rotation3D< T > dx (double t) const
        {
            /*rw::math::Rotation3D< T > rot = x (1.0);
            return inverse (_start) * rot;*/
            rw::math::EAA<T> change(_start.inverse(true) * _end);
            
            return rw::math::EAA<T> (change.axis(),change.angle()/_duration).toRotation3D();
        }

        /**
         * @cond
         * @copydoc Interpolator::ddx()
         * @endcond
         */
        rw::math::Rotation3D< T > ddx (double t) const
        {
            return rw::math::Rotation3D<T>::identity ();
            // return InterpolatorUtil::vecToTrans<V,T>(_interpolator.ddx(t));
        }

        /**
         * @brief Returns the start rotation of the interpolator
         * @return The start rotation of the interpolator
         */
        rw::math::Rotation3D< T > getStart () const { return _start; }

        /**
         * @brief Returns the end rotation of the interpolator
         * @return The end rotation of the interpolator
         */
        rw::math::Rotation3D< T > getEnd () const { return _end; }

        /**
         * @cond
         * @copydoc Interpolator::duration()
         * @endcond
         */
        double duration () const { return _duration; }

      private:
        rw::math::Rotation3D< T > _start;
        rw::math::Rotation3D< T > _end;
        rw::math::Quaternion< T > _quarStart;
        rw::math::Quaternion< T > _quarEnd;

        double _duration;
    };

    /**
     * @brief Implements LinearInterpolator for rw::math::Transform3D<T>
     *
     * The interpolation of rotation is made using Quaternions.
     */
    template< class T >
    class LinearInterpolator< rw::math::Transform3D< T > >
        : public Interpolator< rw::math::Transform3D< T > >
    {
      public:
        //! @brief smart pointer type to this class
        typedef typename rw::core::Ptr< LinearInterpolator< rw::math::Transform3D< T > > > Ptr;

        //! @brief smart pointer type const instance of class
        typedef typename rw::core::Ptr< const LinearInterpolator< rw::math::Transform3D< T > > >
            CPtr;

        /**
         * @brief Construct LinearInterpolator starting a \b start and finishing in \b end
         * and taking \b duration time.
         *
         * If \b duration <= 0 an exception is thrown
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        LinearInterpolator (const rw::math::Transform3D< T >& start,
                            const rw::math::Transform3D< T >& end, double duration) :
            _start (start),
            _end (end), _posInterpolator (start.P (), end.P (), duration),
            _rotInterpolator (start.R (), end.R (), duration)
        {}

        /**
         * @cond
         * @copydoc Interpolator::x()
         * @endcond
         */
        rw::math::Transform3D< T > x (double t) const
        {
            return rw::math::Transform3D< T > (_posInterpolator.x (t), _rotInterpolator.x (t));
            // std::cout<<"Pose = "<<_interpolator.x(t)<<std::endl;
            // return InterpolatorUtil::vecToTrans<V,T>(_interpolator.x(t));
        }

        /**
         * @cond
         * @copydoc Interpolator::dx()
         * @endcond
         */
        rw::math::Transform3D< T > dx (double t) const
        {
            return rw::math::Transform3D< T > (_posInterpolator.dx (t), _rotInterpolator.dx (t));
            // return InterpolatorUtil::vecToTrans<V,T>(_interpolator.dx(t));
        }

        /**
         * @cond
         * @copydoc Interpolator::ddx()
         * @endcond
         */
        rw::math::Transform3D< T > ddx (double t) const
        {
            return rw::math::Transform3D< T > (_posInterpolator.ddx (t), _rotInterpolator.ddx (t));
            // return InterpolatorUtil::vecToTrans<V,T>(_interpolator.ddx(t));
        }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        rw::math::Transform3D< T > getStart () const { return _start; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        rw::math::Transform3D< T > getEnd () const { return _end; }

        /**
         * @cond
         * @copydoc Interpolator::duration()
         * @endcond
         */
        double duration () const { return _posInterpolator.duration (); }

        /**
         * @brief Returns LinearInterpolator for the position part of the transform
         *
         * @note This method is needed by ParabolicBlend
         */
        const LinearInterpolator< rw::math::Vector3D< T > >& getPositionInterpolator () const
        {
            return _posInterpolator;
        }

        /**
         * @brief Returns LinearInterpolator for the rotation part of the transform
         *
         * @note This method is needed by ParabolicBlend
         */
        const LinearInterpolator< rw::math::Rotation3D< T > >& getRotationInterpolator () const
        {
            return _rotInterpolator;
        }

      private:
        rw::math::Transform3D< T > _start;
        rw::math::Transform3D< T > _end;
        LinearInterpolator< rw::math::Vector3D< T > > _posInterpolator;
        LinearInterpolator< rw::math::Rotation3D< T > > _rotInterpolator;
    };

    /**
     * @brief LinearInterpolator with T=rw:math::Q
     */
    typedef LinearInterpolator< rw::math::Q > QLinearInterpolator;

    /**
     * @brief LinearInterpolator with T=rw:math::Transform3D<>
     */
    typedef LinearInterpolator< rw::math::Transform3D<> > CartesianLinearInterpolator;

    /** @} */

}}    // namespace rw::trajectory

#endif    // end include guard
