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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICFACE_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICFACE_HPP_

/**
 * @file QuadraticFace.hpp
 *
 * \copydoc rw::geometry::QuadraticFace
 */
#if !defined(SWIG)
#include "QuadraticCurve.hpp"
#include "QuadraticSurface.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/geometry/analytic/Face.hpp>
#include <rw/math/Vector3D.hpp>
#endif
namespace rw { namespace geometry {

    class QuadraticCurve;
    class TriMesh;

    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief A Quadratic surface patch bounded by Quadratic curves.
     */
#if !defined(SWIGJAVA)
    class QuadraticFace : public Face
#else
    class QuadraticFace : public rw::geometry::GeometryData
#endif
    {
      public:
        //! @brief Smart pointer type to QuadraticFace
        typedef rw::core::Ptr< QuadraticFace > Ptr;

        //! @brief Smart pointer type to const QuadraticFace
        typedef rw::core::Ptr< const QuadraticFace > CPtr;

        //! @brief Constructor.
        QuadraticFace ();

        /**
         * @brief Construct face with surface and vertices given initially.
         *
         * Curves must be set afterwards.
         *
         * @param surface [in] the surface data.
         * @param vertices [in] vector of vertices.
         */
        QuadraticFace (rw::core::Ptr< const rw::geometry::QuadraticSurface > surface,
                       const std::vector< rw::math::Vector3D<double> >& vertices);

        //! @brief Destructor.
        virtual ~QuadraticFace ();

        //! @copydoc Face::surface
        virtual const rw::geometry::QuadraticSurface& surface () const;

        //! @copydoc Face::curveCount
        virtual std::size_t curveCount () const { return _curves.size (); }

        //! @copydoc Face::getCurve
        virtual const rw::geometry::QuadraticCurve& getCurve (std::size_t i) const;

        //! @copydoc Face::vertices
        virtual const std::vector< rw::math::Vector3D<double> >& vertices () const { return _vertices; }

        //! @copydoc Face::transform(const rw::math::Vector3D<double>&)
        virtual void transform (const rw::math::Vector3D<double>& P);

        //! @copydoc Face::transform(const rw::math::Transform3D<>&)
        virtual void transform (const rw::math::Transform3D<>& T);

        /**
         * @brief Get the Quadratic curves.
         * @return vector with the curves.
         */
        const std::vector< rw::core::Ptr< const rw::geometry::QuadraticCurve > >& getCurves () const
        {
            return _curves;
        }

        /**
         * @brief Set Quadratic surface.
         * @param surface [in] the surface.
         */
        void setSurface (rw::core::Ptr< const rw::geometry::QuadraticSurface > surface)
        {
            _surface = surface;
        }

        /**
         * @brief Set surface.
         * @param surface [in] the surface.
         */
        void setSurface (const rw::geometry::QuadraticSurface& surface);

        /**
         * @brief Set Quadratic curve (a curve has direction)
         * @param vertex [in] the start vertex.
         * @param curve [in] the curve.
         */
        void setCurve (std::size_t vertex,
                       rw::core::Ptr< const rw::geometry::QuadraticCurve > curve);

        /**
         * @brief Set the Quadratic curves.
         * @param curves [in] vector of directed curves.
         */
        void setCurves (
            const std::vector< rw::core::Ptr< const rw::geometry::QuadraticCurve > >& curves);

        /**
         * @brief Set vertex.
         * @param index [in] vertex index to set.
         * @param vertex [in] the vertex point.
         */
        void setVertex (std::size_t index, const rw::math::Vector3D<double>& vertex);

        /**
         * @brief Set the vertices.
         * @param vertices [in] vector of vertices.
         */
        void setVertices (const std::vector< rw::math::Vector3D<double> >& vertices);

      private:
        rw::core::Ptr< const rw::geometry::QuadraticSurface > _surface;
        std::vector< rw::core::Ptr< const rw::geometry::QuadraticCurve > > _curves;
        std::vector< rw::math::Vector3D<double> > _vertices;
    };
#if !defined(SWIG)
//! @}
#endif
}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICFACE_HPP_ */
