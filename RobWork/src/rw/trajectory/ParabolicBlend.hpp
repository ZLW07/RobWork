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

#ifndef RW_TRAJECTORY_PARABOLICBLEND_HPP
#define RW_TRAJECTORY_PARABOLICBLEND_HPP

/**
 * @file ParabolicBlend.hpp
 */

#include "Blend.hpp"
#include "LinearInterpolator.hpp"

#include <rw/core/macros.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Implements a parabolic blend
     *
     * A parabolic blend is characterized by a constant acceleration through the blend. The current
     * implementation only supports blending between linear segments.
     */
    template< class T > class ParabolicBlend : public Blend< T >
    {
      public:
        /**
         * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
         * as blend time
         * @param line1 [in] First segment
         * @param line2 [in] Second segment
         * @param tau [in] Blend time
         */
        ParabolicBlend (const LinearInterpolator< T >& line1, const LinearInterpolator< T >& line2,
                        double tau) : _tau(tau)
        {
            if (_tau > line1.duration () || _tau > line2.duration ()) {
                RW_THROW ("blend time tau is greater then LineInterpolator duration");
            }
            if ((line1.getEnd () - line2.getStart ()).norm2() > 10e-10) {
                RW_THROW ("Bland cannot be performed when line1.getEnd() != line2.getStart()");
            }
            double T1 = line1.duration ();
            T X       = line1.getEnd ();
            T v1      = (line1.getStart () - line1.getEnd ()) / (0 - line1.duration ());
            T v2      = (line2.getEnd () - line1.getEnd ()) / (line2.duration ());

            T K       = (v2 - v1) / (4 * tau);
            double K2 = -T1 + tau;
            T& K3     = v1;
            double K4 = -T1;
            T& K5     = X;

            _a = K;
            _b = 2 * K * K2 + K3;
            _c = K * pow (K2, 2) + K3 * K4 + K5;

            _duration = line1.duration () + line2.duration ();
            _start    = line1.duration () - _tau;
        }

        /**
         * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
         * as blend time. The segments is copied for internal storage
         * @param line1 [in] First segment
         * @param line2 [in] Second segment
         * @param tau [in] Blend time
         */
        explicit ParabolicBlend (const typename LinearInterpolator< T >::CPtr line1,
                                 const typename LinearInterpolator< T >::CPtr line2, double tau) :
            ParabolicBlend (*line1, *line2, tau)
        {}

        /**
         * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
         * as blend time. The segments is copied for internal storage
         * @param line1 [in] First segment
         * @param line2 [in] Second segment
         * @param tau [in] Blend time
         */
        ParabolicBlend (const typename LinearInterpolator< T >::Ptr line1,
                        const typename LinearInterpolator< T >::Ptr line2, double tau) :
            ParabolicBlend (*line1, *line2, tau)
        {}

        /**
         * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
         * as blend time. The segments is copied for internal storage
         * @param line1 [in] First segment.
         * @param line2 [in] Second segment
         * @param tau [in] Blend time
         */
        ParabolicBlend (const LinearInterpolator< T >* line1, const LinearInterpolator< T >* line2,
                        double tau) :
            ParabolicBlend (*line1, *line2, tau)
        {}

        /**
         * @brief Destructor
         */
        virtual ~ParabolicBlend () {}

        /**
         * @cond
         * @copydoc Blend::x
         * @endcond
         */
        virtual T x (double t) const
        {
            t = t + _start;
            return _a * t * t + _b * t + _c;
        }

        /**
         * @cond
         * @copydoc Blend::dx
         * @endcond
         */
        virtual T dx (double t) const
        {
            t = t + _start;
            return 2 * _a * t + _b;
        }

        /**
         * @cond
         * @copydoc Blend::ddx
         * @endcond
         */
        virtual T ddx (double t) const { return 2 * _a; }

        /**
         * @brief get the duration of the blend
         * @return duration
         */
        double duration () const { return _duration; }

        /**
         * @cond
         * @copydoc Blend::tau1()
         * @endcond
         *
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau1 () const { return _tau; }

        /**
         * @cond
         * @copydoc Blend::tau2()
         * @endcond
         *
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau2 () const { return _tau; }

      private:
        double _tau;
        double _duration;
        double _start;

        T _a;
        T _b;
        T _c;
    };

    /**
     * @brief Template specialization of ParabolicBlend for using a rw::math::Rotation3D<T>
     *
     * The rotation is blended by calculating equivalent angle axis rotations and blend
     * between these with an ordinary parabolic blend.
     */
    template< class T >
    class ParabolicBlend< rw::math::Rotation3D< T > > : public Blend< rw::math::Rotation3D< T > >
    {
      private:
        ParabolicBlend< rw::math::Vector3D< T > >
        getBlend (const typename LinearInterpolator< rw::math::Rotation3D< T > >::CPtr line1,
                  const typename LinearInterpolator< rw::math::Rotation3D< T > >::CPtr line2,
                  double tau)
        {
            const rw::math::Rotation3D< T > rotStart = line1->x (line1->duration () - tau);
            const rw::math::Rotation3D< T > rotBlend = line1->getEnd ();
            const rw::math::Rotation3D< T > rotEnd   = line2->x (tau);

            const rw::math::EAA< T > u1 (inverse (rotBlend) * rotStart);
            const rw::math::EAA< T > u2 (inverse (rotBlend) * rotEnd);

            const LinearInterpolator< rw::math::Vector3D< T > > lin1 (
                u1.axis () * u1.angle (), rw::math::Vector3D<> (0, 0, 0), tau);
            const LinearInterpolator< rw::math::Vector3D< T > > lin2 (
                rw::math::Vector3D< T > (0, 0, 0), u2.axis () * u2.angle (), tau);
            // The Parabolic blend only read values from the arguments and does not store them
            // (therefore it is ok with the local parameters)
            return ParabolicBlend< rw::math::Vector3D< T > > (&lin1, &lin2, tau);
        }

      public:
        /**
         * @brief Constructs a ParabolicBlend for blending the rotation between \b int1 and \b int2.
         *
         * @param line1 [in] LinearInterpolator representing the first rotational segment
         * @param line2 [in] LinearInterpolator representing the second rotational segment
         * @param tau [in] The blend time
         */
        ParabolicBlend (typename LinearInterpolator< rw::math::Rotation3D< T > >::CPtr line1,
                        typename LinearInterpolator< rw::math::Rotation3D< T > >::CPtr line2,
                        double tau) :
            _blend (getBlend (line1, line2, tau)),
            _blendRot (line1->getEnd ())
        {
            _duration = line1->duration () + line2->duration ();
        }

        /**
         * @brief Destructor
         */
        virtual ~ParabolicBlend () {}

        /**
         * @cond
         * @copydoc Blend::x
         * @endcond
         */
        rw::math::Rotation3D< T > x (double t) const
        {
            rw::math::Vector3D< T > v = _blend.x (t);
            rw::math::EAA< T > eaa (v (0), v (1), v (2));
            return _blendRot * eaa.toRotation3D ();
        }

        /**
         * @cond
         * @copydoc Blend::dx
         * @endcond
         */
        rw::math::Rotation3D<> dx (double t) const
        {
            rw::math::Vector3D< T > v = _blend.x (t);
            rw::math::EAA< T > eaa (v (0), v (1), v (2));
            return eaa.toRotation3D ();
        }

        /**
         * @cond
         * @copydoc Blend::ddx
         * @endcond
         */
        rw::math::Rotation3D<> ddx (double t) const
        {
            rw::math::Vector3D< T > v = _blend.x (t);
            rw::math::EAA< T > eaa (v (0), v (1), v (2));
            return eaa.toRotation3D ();
        }

        /**
         * @brief get the duration of the blend
         * @return duration
         */
        double duration () const { return _duration; }

        /**
         * @brief get the deviation from the intersection point
         * @return the deviation
         */
        rw::math::Rotation3D<> deviation () const
        {
            RW_THROW ("deviation is not implemented for rotation");
            return rw::math::Rotation3D<> ();
        }

        /**
         * @cond
         * @copydoc Blend::tau1()
         * @endcond
         *
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau1 () const { return _blend.tau1 (); }

        /**
         * @cond
         * @copydoc Blend::tau1()
         * @endcond
         *
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau2 () const { return _blend.tau2 (); }

      private:
        ParabolicBlend< rw::math::Vector3D< T > > _blend;
        rw::math::Rotation3D< T > _blendRot;
        double _duration;
    };

    /**
     * @brief Template specialization of ParabolicBlend for using a rw::math::Transform3D<T>
     *
     * The transform is encoded as a vector storing the position and the orientation as a
     * quaternion.
     */
    template< class T >
    class ParabolicBlend< rw::math::Transform3D< T > > : public Blend< rw::math::Transform3D< T > >
    {
      public:
        /**
         * @brief Constructs parabolic blend between \b line1 and \b line2 with \b tau
         * as blend time
         * @param line1 [in] First segment
         * @param line2 [in] Second segment
         * @param tau [in] Blend time
         */
        ParabolicBlend (typename LinearInterpolator< rw::math::Transform3D< T > >::CPtr line1,
                        typename LinearInterpolator< rw::math::Transform3D< T > >::CPtr line2,
                        double tau) :
            _posBlend (&(line1->getPositionInterpolator ()), &(line2->getPositionInterpolator ()),
                       tau),
            _rotBlend (&(line1->getRotationInterpolator ()), &(line2->getRotationInterpolator ()),
                       tau)
        {
            _duration = line1->duration () + line2->duration ();
        }

        /**
         * @brief Destructor
         */
        virtual ~ParabolicBlend () {}

        /**
         * @cond
         * @copydoc Blend::x
         * @endcond
         */
        rw::math::Transform3D< T > x (double t) const
        {
            rw::math::Vector3D< T > pos   = _posBlend.x (t);
            rw::math::Rotation3D< T > rot = _rotBlend.x (t);
            rw::math::Transform3D< T > result (pos, rot);
            return result;
            // return InterpolatorUtil::vecToTrans<V,T>(v);
        }

        /**
         * @cond
         * @copydoc Blend::dx
         * @endcond
         */
        rw::math::Transform3D<> dx (double t) const
        {
            rw::math::Vector3D< T > pos   = _posBlend.dx (t);
            rw::math::Rotation3D< T > rot = _rotBlend.dx (t);
            rw::math::Transform3D< T > result (pos, rot);
            return result;
            //        return InterpolatorUtil::vecToTrans<V,T>(v);
        }

        /**
         * @cond
         * @copydoc Blend::ddx
         * @endcond
         */
        rw::math::Transform3D<> ddx (double t) const
        {
            rw::math::Vector3D< T > pos   = _posBlend.ddx (t);
            rw::math::Rotation3D< T > rot = _rotBlend.ddx (t);
            rw::math::Transform3D< T > result (pos, rot);
            return result;

            //        V v = _blend.ddx(t);
            //      return InterpolatorUtil::vecToTrans<V,T>(v);
        }

        /**
         * @brief get the duration of the blend
         * @return duration
         */
        double duration () const { return _duration; }

        /**
         * @brief get the deviation from the intersection point
         * @return the deviation
         */
        rw::math::Transform3D<> deviation () const
        {
            RW_THROW ("deviation is not implemented for transform");
            return rw::math::Transform3D<> ();
        }

        /**
         * @cond
         * @copydoc Blend::tau1()
         * @endcond
         *
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau1 () const { return _posBlend.tau1 (); }

        /**
         * @cond
         * @copydoc Blend::tau1()
         * @endcond
         * @note For ParabolicBlend tau1()==tau2()
         */
        double tau2 () const { return _posBlend.tau2 (); }

      private:
        // ParabolicBlend<V> _blend;

        ParabolicBlend< rw::math::Vector3D< T > > _posBlend;
        ParabolicBlend< rw::math::Rotation3D< T > > _rotBlend;
        double _duration;
    };

    /** @} */

}}    // namespace rw::trajectory

#endif    // RW_TRAJECTORY_PARABOLICBLEND_HPP
