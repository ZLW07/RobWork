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

#ifndef RW_GEOMETRY_POLYGON_HPP_
#define RW_GEOMETRY_POLYGON_HPP_

/**
 * @file Polygon.hpp
 *
 * \copydoc rw::geometry::Polygon
 */

#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/core/macros.hpp>
#include <rw/math/Vector3D.hpp>
#endif

namespace rw { namespace geometry {
    //! @addtogroup geometry
    // @{

    /**
     * @brief indexed polygon class that saves \b N indices to the \b N vertices of the polygon
     */
    template< class T = rw::math::Vector3D< double > > class Polygon
    {
      public:
        //! @brief Smart pointer to Polygon
        typedef rw::core::Ptr< Polygon< T > > Ptr;

        //! @brief value type of the index pointer
        typedef T value_type;

        /**
         * @brief Adds a vertex to the polygon
         *
         * The point will be added to the end of the list of points
         * @param p [in] The point to add
         */
        void addVertex (const T& p) { _vertices.push_back (p); }

        /**
         * @brief Removes vertex from the polygon
         *
         * @param idx [in] Index of the vertex to remove
         */
        void removeVertex (size_t idx)
        {
            RW_ASSERT_MSG (idx < _vertices.size (),
                           "The requested index "
                               << idx
                               << " is not less than the number of items: " << _vertices.size ());
            _vertices.erase (_vertices.begin () + idx);
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        T& getVertex (size_t idx)
        {
            RW_ASSERT_MSG (idx < _vertices.size (),
                           "The requested index "
                               << idx
                               << " is not less than the number of items: " << _vertices.size ());
            return _vertices[idx];
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertex (size_t idx) const
        {
            RW_ASSERT_MSG (idx < _vertices.size (),
                           "The requested index "
                               << idx
                               << " is not less than the number of items: " << _vertices.size ());
            return _vertices[idx];
        }
#if !defined(SWIG)
        /**
         * @brief get vertex at index i
         */
        T& operator[] (size_t i) { return getVertex (i); }

        /**
         * @brief get vertex at index i
         */
        const T& operator[] (size_t i) const { return getVertex (i); }
#else
        ARRAYOPERATOR (T);
#endif
        /**
         * @brief Number of vertices of this polygon
         * @return Number of vertices
         */
        size_t size () const { return _vertices.size (); }

        /**
         * @brief Computes the center of the polygon as the average of all coordinates
         * @return Center of the polygon
         */
        T computeCenter ()
        {
            T sum;
            for (size_t i = 0; i < _vertices.size (); i++) {
                sum += _vertices[i];
            }
            return sum / (double) _vertices.size ();
        }

      protected:
        /**
         * @brief Vertices making up the polygon
         */
        std::vector< T > _vertices;
    };
#if defined(SWIG)
#if SWIG_VERSION < 0x040000
    SWIG_DECLARE_TEMPLATE (Polygon_d, rw::geometry::Polygon< rw::math::Vector3D< double > >);
    SWIG_DECLARE_TEMPLATE (Polygon2D_d, rw::geometry::Polygon< rw::math::Vector2D< double > >);
    ADD_DEFINITION (Polygon_d, Polygon)
    ADD_DEFINITION (Polygon2D_d, Polygon2D)
#else
    SWIG_DECLARE_TEMPLATE (Polygon, rw::geometry::Polygon< rw::math::Vector3D< double > >);
    SWIG_DECLARE_TEMPLATE (Polygon2D, rw::geometry::Polygon< rw::math::Vector2D< double > >);
#endif

    SWIG_DECLARE_TEMPLATE (Polygon_f, rw::geometry::Polygon< rw::math::Vector3D< float > >);
    SWIG_DECLARE_TEMPLATE (Polygon2D_f, rw::geometry::Polygon< rw::math::Vector2D< float > >);
#endif
    // @}
}}    // namespace rw::geometry

#endif /*RW_GEOMETRY_POLYGON_HPP_*/
