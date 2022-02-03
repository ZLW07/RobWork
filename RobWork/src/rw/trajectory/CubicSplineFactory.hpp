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

#ifndef RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP
#define RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP

/**
 * @file CubicSplineFactory.hpp
 */

#include "InterpolatorTrajectory.hpp"
#include "Path.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Transform3DVector.hpp>
#include <rw/math/Quaternion.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Factory for creating cubic splines
     *
     *
     */
    class CubicSplineFactory
    {
      public:
      
        /**
         * @brief constructs a free/natural cubic spline
         * A natural cubic spline has free boundary conditions. Only one condition
         * can be said for the end points namely acceleration is zero.
         * * The spline passes through each data point.
         * * The spline forms a continuous function over [a,b].
         * * The spline forms a smooth function.
         * * The second derivative is continuous.
         *
         * @param qpath [in] a list of points that the spline should intersect
         * @param timeStep [in] the duration of each spline path
         */
        static InterpolatorTrajectory< rw::math::Q >::Ptr makeNaturalSpline (QPath::Ptr qpath,
                                                                             double timeStep = 1.0);

        /**
         * @brief constructs a natural cubic spline, see above.
         *
         * @param tqpath [in] a list of points with associated timestaps. The spline will intersect
         * the points at the time specified in \b tqpath
         * @cond
         * @param offset [in]
         * @endcond
         * @return a trajectory of CubicSplineInterpolators
         */
        static InterpolatorTrajectory< rw::math::Q >::Ptr
        makeNaturalSpline (TimedQPath::Ptr tqpath);

        /**
         * @brief Construct a natural cubic spline. See documentation of
         * CubicSplineFactory::makeNaturalSpline(QPath::Ptr, double)
         *
         * @param qpath [in] Path to follow
         * @param times [in] Times associated to the different configurations in \b qpath
         * @return a trajectory of CubicSplineInterpolators
         */
        static InterpolatorTrajectory< rw::math::Q >::Ptr
        makeNaturalSpline (const QPath& qpath, const std::vector< double >& times);

        /**
         * @brief constructs a free/natural cubic spline
         * A natural cubic spline has free boundary conditions. Only one condition
         * can be said for the end points namely acceleration is zero.
         * * The spline passes through each data point.
         * * The spline forms a continuous function over [a,b].
         * * The spline forms a smooth function.
         * * The second derivative is continuous.
         *
         * @param path [in] a list of points that the spline should intersect
         * @param timeStep [in] the duration of each spline path
         */
        static InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
        makeNaturalSpline (const Path< rw::math::Transform3D<> >& path, double timeStep = 1.0);

        /**
         * @brief constructs a natural cubic spline, see above.
         * @param path [in] a list of points with associated timestaps. The spline will intersect
         * the points at the time specified in \b tqpath
         * @cond
         * @param offset [in]
         * @endcond
         * @return a trajectory of CubicSplineInterpolators
         */
        static InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
        makeNaturalSpline (const Path< Timed< rw::math::Transform3D<> > >& path);

        /**
         * @brief Construct a natural cubic spline. See documentation of
         * CubicSplineFactory::makeNaturalSpline(QPath::Ptr, double)
         *
         * @param path [in] Path to follow
         * @param times [in] Times associated to the different configurations in \b path
         * @return a trajectory of CubicSplineInterpolators
         */
        static InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
        makeNaturalSpline (const Path< rw::math::Transform3D<> >& path,
                           const std::vector< double >& times);

        /**
         * @brief constructs a free/natural cubic spline
         * A natural cubic spline has free boundary conditions. Only one condition
         * can be said for the end points namely acceleration is zero.
         * * The spline passes through each data point.
         * * The spline forms a continuous function over [a,b].
         * * The spline forms a smooth function.
         * * The second derivative is continuous.
         *
         * @param path [in] a list of points that the spline should intersect
         * @param timeStep [in] the duration of each spline path
         * @return the Interpolated Trajectory of the Cubic spline
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr makeNaturalSpline (const Path< T >& path,
                                                                            double timeStep = 1.0);

        /**
         * @brief constructs a natural cubic spline, see above.
         * @param path [in] a list of points with associated timestaps. The spline will intersect
         * the points at the time specified in \b tqpath
         * @cond
         * @param offset [in]
         * @endcond
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr
        makeNaturalSpline (const Path< Timed< T > >& path);

        /**
         * @brief Construct a natural cubic spline. See documentation of
         * CubicSplineFactory::makeNaturalSpline(QPath::Ptr, double)
         *
         * @param path [in] Path to follow
         * @param times [in] Times associated to the different configurations in \b path
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr
        makeNaturalSpline (const Path< T >& path, const std::vector< double >& times);

        /**
         * @brief creates a clamped spline trajectory with equally spaced
         * via points. That is time between samples is 1. A clamped spline controls
         * the velocity in the end points. The acceleration is 0 in the end points.
         * @param qpath [in] the path over which the spline should be generated.
         * @param dqStart [in] the velocity in the first point
         * @param dqEnd [in] the velocity in the last point.
         * @param timeStep documentation missing !
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        static InterpolatorTrajectory< rw::math::Q >::Ptr
        makeClampedSpline (QPath::Ptr qpath, const rw::math::Q& dqStart, const rw::math::Q& dqEnd,
                           double timeStep = 1.0);

        /**
         * @brief creates a clamped spline trajectory where the timed label is used
         * to determine the time between samples. A clamped spline controls
         * the velocity in the end points. The acceleration is 0 in the end points.
         * @param tqpath [in] the path over which the spline should be generated.
         * @param dqStart [in] the velocity in the first point
         * @param dqEnd [in] the velocity in the last point.
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        static InterpolatorTrajectory< rw::math::Q >::Ptr
        makeClampedSpline (TimedQPath::Ptr tqpath, const rw::math::Q& dqStart,
                           const rw::math::Q& dqEnd);

        /**
         * @brief creates a clamped spline trajectory with equally spaced
         * via points. That is time between samples is 1. A clamped spline controls
         * the velocity in the end points. The acceleration is 0 in the end points.
         * @param path [in] the path over which the spline should be generated.
         * @param dStart [in] the velocity in the first point
         * @param dEnd [in] the velocity in the last point.
         * @param timeStep documentation missing !
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr
        makeClampedSpline (const Path< T >& path, const T& dStart, const T& dEnd,
                           double timeStep = 1.0);

        /**
         * @brief creates a clamped spline trajectory where the timed label is used
         * to determine the time between samples. A clamped spline controls
         * the velocity in the end points. The acceleration is 0 in the end points.
         * @param tqpath [in] the path over which the spline should be generated.
         * @param dqStart [in] the velocity in the first point
         * @param dqEnd [in] the velocity in the last point.
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr
        makeClampedSpline (const Path< Timed< T > >& tpath, const T& dStart, const T& dEnd);

        /**
         * @brief creates a clamped spline trajectory where the timed label is used
         * to determine the time between samples. A clamped spline controls
         * the velocity in the end points. The acceleration is 0 in the end points.
         * @param qpath [in] the path over which the spline should be generated.
         * @param times [in] the times associated to the configurations in \b qpath.
         * @param dqStart [in] the velocity in the first point
         * @param dqEnd [in] the velocity in the last point.
         * @return a trajectory of CubicSplineInterpolators
         * @note the following template parameters are currently supported:
         * Transform3DVector, Vector3D, Quaternion
         */
        template< typename T >
        static typename InterpolatorTrajectory< T >::Ptr
        makeClampedSpline (const Path< T >& path, const std::vector< double >& times,
                           const T& dStart, const T& dEnd);

        /**
         * @brief constructs a Spherical Spline Quaternion interpolation (SQUAD)
         * A natural SQUAD has free boundary conditions.
         * The SQUAD passes through each data point.
         * The SQUAD forms a continuous function over [a,b].
         * The SQUAD forms a smooth function.
         * The second derivative is continuous.
         *
         * @param path [in] a list of points that the SQUAD should intersect
         * @param timeStep [in] the duration of each SQUAD path
         * @return a trajectory of SQUADInterpolators
         */
        static InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
        makeSQUAD (const Path< rw::math::Quaternion<> >& path, double timeStep = 1.0);

        /**
         * @brief constructs a Spherical Spline Quaternion interpolation (SQUAD)
         *
         * @param tpath [in] a list of points with associated timestaps. The SQUAD will intersect
         * the points at the time specified in \b tpath
         * @return a trajectory of SQUADInterpolators
         */
        static InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
        makeSQUAD (const Path< Timed< rw::math::Quaternion<> > >& tpath);

        /**
         * @brief constructs a Spherical Spline Quaternion interpolation (SQUAD)
         *
         * @param path [in] Path to follow
         * @param times [in] Times associated to the different configurations in \b qpath
         * @return a trajectory of CubicSplineInterpolators
         */
        static InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
        makeSQUAD (const Path< rw::math::Quaternion<> >& path, const std::vector< double >& times);

      private:
        CubicSplineFactory ();

        virtual ~CubicSplineFactory ();
    };

    /** @} */

}}    // namespace rw::trajectory

#endif /*RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP_*/
