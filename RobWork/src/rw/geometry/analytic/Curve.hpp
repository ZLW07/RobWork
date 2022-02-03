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

#ifndef RW_GEOMETRY_ANALYTIC_CURVE_HPP_
#define RW_GEOMETRY_ANALYTIC_CURVE_HPP_

/**
 * @file Curve.hpp
 *
 * \copydoc rw::geometry::Curve
 */
#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

#include <list>
#endif
namespace rw { namespace geometry {
    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief Curve is an abstract representation of a smooth curve geometry in 3D.
     *
     * The interface provides functions for affine transformations, such as scaling, rotation and
     * translation. In case of a limited curve segment, it is also possible to make a discretization
     * of the curve into line segments.
     */
    class Curve
    {
      public:
        //! @brief Smart pointer type for Curve.
        typedef rw::core::Ptr< Curve > Ptr;

        //! @brief Smart pointer type for a const Curve.
        typedef rw::core::Ptr< const Curve > CPtr;

        //! @brief Constructor.
        Curve () {}

        //! @brief Destructor.
        virtual ~Curve () {}

        /**
         * @brief Transform curve.
         * @param T [in] transformation of curve.
         * @return a new transformed curve.
         */
        inline rw::core::Ptr<Curve> transform (const rw::math::Transform3D< double >& T) const
        {
            return doTransformCurve (T);
        }

        /**
         * @brief Transform curve.
         * @param P [in] positional offset.
         * @return a new transformed curve.
         */
        inline rw::core::Ptr<Curve> transform (const rw::math::Vector3D< double >& P) const
        {
            return doTransformCurve (P);
        }

        /**
         * @brief Get a scaled version of the curve.
         * @param factor [in] the factor to scale with.
         * @return a new scaled curve.
         */
        inline rw::core::Ptr<Curve> scale (double factor) const { return doScaleCurve (factor); }

        /**
         * @brief Make a curve where time variable runs in opposite direction.
         * @return reversed curve.
         */
        inline rw::core::Ptr<Curve> reverse () const { return doReverseCurve (); }

        /**
         * @brief Make a copy of the curve.
         * @return a new copy of the curve.
         */
        inline rw::core::Ptr<Curve> clone () const { return doCloneCurve (); }

        /**
         * @brief Get extremums of curve in given direction.
         *
         * Notice that the limits are taken into account.
         *
         * @param dir [in] direction to get extremums for.
         * @return the minimum and maximum value of the curve in the given direction.
         */
        virtual std::pair< double, double > extremums (const rw::math::Vector3D<double>& dir) const = 0;

        /**
         * @brief Make a discretization of the curve.
         *
         * The curve must be limited. The discretization is based on the curvature, such
         * that the sampling starts at maximum curvature points (or in limits). The step
         * size to the next point is based on the curvature in the current point.
         *
         * A line will always give to points, regardless of the chosen number of steps per
         * revolution.
         *
         * @param stepsPerRevolution [in] the number of points to sample if the curve is a perfect
         * circle.
         * @return a list of points on the curve.
         */
        virtual std::list< rw::math::Vector3D<double > >
        discretizeAdaptive (double stepsPerRevolution) const = 0;

        /**
         * @brief Bounding rectangle of curve.
         *
         * The curve must be limited.
         *
         * @return the bounding rectangle.
         */
        virtual OBB<> obr () const = 0;

        /**
         * @brief Get the closest points on the curve to a point \b p.
         *
         * Notice that the limits are taken into account.
         *
         * @param p [in] the point to find closest values for.
         * @return a vector of closest points to \b p.
         */
        virtual std::vector< rw::math::Vector3D<double> >
        closestPoints (const rw::math::Vector3D<double>& p) const = 0;

        /**
         * @brief Check if this curve is equal to another curve.
         * @param curve [in] other curve.
         * @param eps [in] distance threshold.
         * @return true if curves are identical, false otherwise.
         */
        virtual bool equals (rw::core::Ptr< const Curve> curve, double eps) const = 0;

      private:
        virtual rw::core::Ptr<Curve> doScaleCurve (double factor) const                        = 0;
        virtual rw::core::Ptr<Curve> doTransformCurve (const rw::math::Vector3D<double>& P) const    = 0;
        virtual rw::core::Ptr<Curve> doTransformCurve (const rw::math::Transform3D<>& T) const = 0;
        virtual rw::core::Ptr<Curve> doReverseCurve () const                                   = 0;
        virtual rw::core::Ptr<Curve> doCloneCurve () const                                     = 0;
    };
#if !defined(SWIG)
//! @}
#endif
}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_CURVE_HPP_ */
