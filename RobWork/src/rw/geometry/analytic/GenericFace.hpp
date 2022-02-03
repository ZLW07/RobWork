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

#ifndef RW_GEOMETRY_ANALYTIC_GENERICFACE_HPP_
#define RW_GEOMETRY_ANALYTIC_GENERICFACE_HPP_

/**
 * @file GenericFace.hpp
 *
 * \copydoc rw::geometry::GenericFace
 */

#if !defined(SWIG)
#include "Face.hpp"
#endif
namespace rw { namespace geometry {
    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
      #endif
    /**
     * @brief The GenericFace implementation is a type of Face that consist of abstract Surfaces and
     * Curves.
     */
    class GenericFace : public Face
    {
      public:
        //! @brief Smart pointer type to GenericFace
        typedef rw::core::Ptr< GenericFace > Ptr;

        //! @brief Constructor.
        GenericFace ();

        /**
         * @brief Copy constructor.
         * @param face [in] other face to copy.
         */
        GenericFace (const Face& face);

        //! @brief Destructor.
        virtual ~GenericFace ();

        //! @copydoc Face::surface
        virtual const Surface& surface () const;

        //! @copydoc Face::curveCount
        virtual std::size_t curveCount () const { return _curves.size (); }

        //! @copydoc Face::getCurve
        virtual const Curve& getCurve (std::size_t i) const;

        //! @copydoc Face::vertices
        virtual const std::vector< rw::math::Vector3D<double> >& vertices () const { return _vertices; }

        //! @copydoc Face::transform(const rw::math::Vector3D<double>&)
        virtual void transform (const rw::math::Vector3D<double>& P);

        //! @copydoc Face::transform(const rw::math::Transform3D<>&)
        virtual void transform (const rw::math::Transform3D<>& T);

        /**
         * @brief Set surface.
         * @param surface [in] the surface.
         */
        void setSurface (rw::core::Ptr< const Surface > surface) { _surface = surface; }

        /**
         * @brief Set surface.
         * @param surface [in] the surface.
         */
        void setSurface (const Surface& surface);

        /**
         * @brief Set curve (a curve has direction)
         * @param vertex [in] the start vertex.
         * @param curve [in] the curve.
         */
        void setCurve (std::size_t vertex, rw::core::Ptr< const Curve > curve);

        /**
         * @brief Set the curves.
         * @param curves [in] vector of directed curves.
         */
        void setCurves (const std::vector< rw::core::Ptr< const Curve > >& curves);

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
        rw::core::Ptr< const Surface > _surface;
        std::vector< rw::core::Ptr< const Curve > > _curves;
        std::vector< rw::math::Vector3D<double> > _vertices;
    };
    #if !defined(SWIG)
    //! @}
    #endif
}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_GENERICFACE_HPP_ */
