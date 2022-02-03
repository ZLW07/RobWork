/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_
#define RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_

/**
 * @file ImplicitBREP.hpp
 *
 * \copydoc rw::geometry::ImplicitBREP
 */

#if !defined(SWIG)
#include "BREP.hpp"
#include "ImplicitSurface.hpp"
#include "ParametricCurve.hpp"
#endif

namespace rw { namespace geometry {

    class ImplicitShell;

    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief Type of BREP where all surfaces are of type ImplicitSurface,
     * and edges are of type ParametricCurve.
     */
#if !defined(SWIGJAVA)
    class ImplicitBREP : public BREP
#else
    class ImplicitBREP : public rw::geometry::GeometryData
#endif
    {
      public:
        //! @brief Smart pointer type to ImplicitBREP
        typedef rw::core::Ptr< ImplicitBREP > Ptr;

        //! @brief Smart pointer type to const ImplicitBREP
        typedef rw::core::Ptr< const ImplicitBREP > CPtr;

        //! @brief Constructor.
        ImplicitBREP ();

        //! @brief Destructor.
        virtual ~ImplicitBREP ();

        //! @copydoc BREP::getType
        virtual GeometryType getType () const;

        //! @copydoc BREP::getSurface
        virtual const rw::geometry::ImplicitSurface& getSurface (std::size_t surfaceIndex) const;

        //! @copydoc BREP::getCurve
        virtual const rw::geometry::ParametricCurve& getCurve (std::size_t curveIndex) const;

        //! @copydoc BREP::scale
        virtual void scale (double factor);

        //! @copydoc BREP::clone
        ImplicitBREP::Ptr clone () const;

        //! @copydoc BREP::shellProxy
        rw::core::Ptr< const rw::geometry::ImplicitShell > shellProxy () const;

        //! @copydoc BREP::getCurves
        std::vector< rw::core::Ptr< rw::geometry::ParametricCurve > >
        getCurves (std::size_t loopIdx) const;

        //! @brief Convenience type for a set of curves in a BREP.
        class CommonParametricCurveSet
        {
          public:
            //! @brief Smart pointer type to CommonParametricCurveSet
            typedef rw::core::Ptr< const CommonParametricCurveSet > CPtr;

            //! @brief Constructor.
            CommonParametricCurveSet () {}

            //! @brief Destructor.
            virtual ~CommonParametricCurveSet () {}

            //! @copydoc BREP::CommonCurveSet::size
            virtual std::size_t size () const = 0;

            //! @copydoc BREP::CommonCurveSet::curve
            virtual const rw::geometry::ParametricCurve& curve (std::size_t index) const = 0;

            //! @copydoc BREP::CommonCurveSet::surfaceLeft
            virtual const rw::geometry::ImplicitSurface& surfaceLeft (std::size_t index) const = 0;

            //! @copydoc BREP::CommonCurveSet::surfaceRight
            virtual const rw::geometry::ImplicitSurface& surfaceRight (std::size_t index) const = 0;
        };

        //! @copydoc BREP::getCommonCurves
        CommonParametricCurveSet::CPtr getCommonCurves (const std::set< std::size_t >& faces) const;

        /**
         * @brief Add a ParametricCurve to the BREP.
         *
         * Notice that the curve has direction. It is expected to have limits such that it starts in
         * vertex \b v1 and end in \b v2.
         *
         * @param curve [in] curve to add.
         * @param v1 [in] the start vertex.
         * @param v2 [in] the end vertex.
         */
        void addEdge (const rw::geometry::ParametricCurve& curve, std::size_t v1, std::size_t v2);

        /**
         * @brief Attach an ImplicitSurface to a face of the BREP.
         * @param surface [in] surface to add.
         * @param loop [in] the loop index for the loop to attach surface to.
         */
        void setFace (const rw::geometry::ImplicitSurface& surface, std::size_t loop);

      protected:
        /**
         * @brief Remove specific curve.
         * @param curveIndex [in] the index of the curve to remove.
         */
        virtual void doRemoveCurve (std::size_t curveIndex);

      private:
        class CommonParametricCurveSetImpl;
        virtual rw::core::Ptr< const Shell > doShellProxyBREP () const;
#if !defined(SWIGJAVA)
        virtual BREP::Ptr doClone () const { return clone (); }
#endif
        // Geometry
        std::vector< rw::geometry::ParametricCurve::CPtr > _curves;
        std::vector< rw::geometry::ImplicitSurface::CPtr > _surfaces;
    };
#if !defined(SWIG)
//! @}
#endif

}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_ */
