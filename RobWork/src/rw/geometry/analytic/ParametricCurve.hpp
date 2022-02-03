/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_
#define RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_

/**
 * @file ParametricCurve.hpp
 *
 * \copydoc rw::geometry::ParametricCurve
 */
#if !defined(SWIG)
#include "Curve.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>
#endif

namespace rw { namespace geometry {
    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
#endif
/**
 * @brief Interface for parametric curves. A parametric curve, \f$ \mathbf{p}(t) \in
 * \mathbb{R}^3 \f$, has its points given as a function of a single parameter, \f$ t \in
 * \mathbb{R}\f$.
 *
 * Parametric curves have functions for evaluation of points, derivatives and curvature.
 * A parmateric curve can be limited, and it is possible to find closest points with a given
 * point.
 */
#if !defined(SWIGJAVA)
    class ParametricCurve : public Curve
#else
    class ParametricCurve
#endif
    {
      public:
        //! @brief Smart pointer type for ParametricCurve.
        typedef rw::core::Ptr< ParametricCurve > Ptr;

        //! @brief Smart pointer type for a const ParametricCurve.
        typedef rw::core::Ptr< const ParametricCurve > CPtr;

        //! @brief Constructor.
        ParametricCurve () {}

        //! @brief Destructor.
        virtual ~ParametricCurve () {}

        //! @copydoc Curve::transform(const rw::math::Transform3D<>&) const
        inline ParametricCurve::Ptr transform (const rw::math::Transform3D<>& T) const
        {
            return doTransformParametricCurve (T);
        }

        //! @copydoc Curve::transform(const rw::math::Vector3D<double>&) const
        inline ParametricCurve::Ptr transform (const rw::math::Vector3D<double>& P) const
        {
            return doTransformParametricCurve (P);
        }

        //! @copydoc Curve::scale
        inline ParametricCurve::Ptr scale (double factor) const
        {
            return doScaleParametricCurve (factor);
        }

        //! @copydoc Curve::reverse
        inline ParametricCurve::Ptr reverse () const { return doReverseParametricCurve (); }

        //! @copydoc Curve::clone
        inline ParametricCurve::Ptr clone () const { return doCloneParametricCurve (); }

        //! @copydoc Curve::extremums
        virtual std::pair< double, double > extremums (const rw::math::Vector3D<double>& dir) const = 0;

        //! @copydoc Curve::discretizeAdaptive
        virtual std::list< rw::math::Vector3D<double> >
        discretizeAdaptive (double stepsPerRevolution) const = 0;

        //! @copydoc Curve::obr
        virtual OBB<> obr () const = 0;

        //! @copydoc Curve::closestPoints
        virtual std::vector< rw::math::Vector3D<double> >
        closestPoints (const rw::math::Vector3D<double>& p) const = 0;

        //! @copydoc Curve::equals
        virtual bool equals (rw::core::Ptr<const rw::geometry::Curve> curve, double eps) const = 0;

        /**
         * @brief Evaluate a point on the curve.
         * @param t [in] the parameter to find point for.
         * @return the vector \f$ p \in \mathbb{R}^3 \f$ .
         */
        virtual rw::math::Vector3D<double> x (double t) const = 0;

        /**
         * @brief Evaluate the derivative in a point on the curve.
         * @param t [in] the parameter to find derivative for.
         * @return a derivative vector \f$ p \in \mathbb{R}^3 \f$ .
         */
        virtual rw::math::Vector3D<double> dx (double t) const = 0;

        /**
         * @brief Evaluate the second derivative in a point on the curve.
         * @param t [in] the parameter to find second derivative for.
         * @return a second derivative vector \f$ p \in \mathbb{R}^3 \f$ .
         */
        virtual rw::math::Vector3D<double> ddx (double t) const = 0;

#if !defined(SWIG)
        //! @copydoc x(double) const
        virtual rw::math::Vector3D<double> operator() (double t) const = 0;
#else 
        CALLOPERATOR(rw::math::Vector3D<double>,double );
#endif 
        /**
         * @brief Check if the curve is limited.
         * @return true if curve is limited, false otherwise.
         */
        virtual bool hasLimits () const = 0;

        /**
         * @brief Get the limits of the curve segment.
         *
         * The returned values are only valid when hasLimits() returns true.
         *
         * @return the minimum and maximum parameter values on the curve.
         */
        virtual const std::pair< double, double >& limits () const = 0;

        /**
         * @brief Check if the parameter \b t is inside the limits set for the curve.
         * @param t [in] the parameter to check.
         * @return true if inside limits, false otherwise.
         */
        virtual bool inLimits (double t) const = 0;

        /**
         * @brief Set parameter limits for the curve.
         * @param limits [in] the minimum and maximum parameter values on the curve.
         */
        virtual void setLimits (const std::pair< double, double >& limits) = 0;

        /**
         * @brief The curvature in a given point on the curve.
         *
         * This function does not take the limits into account.
         *
         * @param t [in] the parameter to evaluate the curvature for.
         * @return the curvature.
         */
        virtual double curvature (double t) const = 0;

        /**
         * @brief Get the parameter values where the curve is closest to a point \b p.
         *
         * Notice that the limits are taken into account.
         *
         * @param p [in] the point to find closest values for.
         * @return a list of parameter values.
         */
        virtual std::vector< double > closestTimes (const rw::math::Vector3D<double>& p) const = 0;

        /**
         * @brief Get the parameter value where the curve is closest to a point \b p.
         *
         * Notice that the limits are taken into account.
         *
         * @param p [in] the point to find closest values for.
         * @return the point on the curve closest to \b p. If multiple points are equally close to
         * \b p, only one of those points are returned.
         */
        virtual double closestTime (const rw::math::Vector3D<double>& p) const = 0;

      private:
#if !defined(SWIGJAVA)
        virtual rw::core::Ptr<rw::geometry::Curve> doScaleCurve (double factor) const
        {
            return doScaleParametricCurve (factor);
        }
        virtual rw::core::Ptr<rw::geometry::Curve> doTransformCurve (const rw::math::Vector3D<double>& P) const
        {
            return doTransformParametricCurve (P);
        }
        virtual rw::core::Ptr<rw::geometry::Curve> doTransformCurve (const rw::math::Transform3D<>& T) const
        {
            return doTransformParametricCurve (T);
        }
        virtual rw::core::Ptr<rw::geometry::Curve> doReverseCurve () const { return doReverseParametricCurve (); }
        virtual rw::core::Ptr<rw::geometry::Curve> doCloneCurve () const { return doCloneParametricCurve (); }
#endif
        virtual ParametricCurve::Ptr doScaleParametricCurve (double factor) const = 0;
        virtual ParametricCurve::Ptr
        doTransformParametricCurve (const rw::math::Vector3D<double>& P) const = 0;
        virtual ParametricCurve::Ptr
        doTransformParametricCurve (const rw::math::Transform3D<>& T) const = 0;
        virtual ParametricCurve::Ptr doReverseParametricCurve () const      = 0;
        virtual ParametricCurve::Ptr doCloneParametricCurve () const        = 0;
    };
#if !defined(SWIG)
//! @}
#endif
}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_ */
