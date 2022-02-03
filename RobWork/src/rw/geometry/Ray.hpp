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

#ifndef RW_GEOMETRY_RAY_HPP
#define RW_GEOMETRY_RAY_HPP

#if !defined(SWIG)
#include "Line.hpp"
#endif

namespace rw { namespace geometry {

    /**
     * @brief A ray - an infinitely extending half-line described by a starting position and a
     * direction.
     */
    class Ray : public Line
    {
      public:
        /**
         * @brief constructor
         * @param pos [in] position from which the ray starts
         * @param dir [in] direction in which the ray shoots
         */
        Ray (rw::math::Vector3D<double>& pos, rw::math::Vector3D<double>& dir) :
            Line (pos, pos + dir), _pos (pos), _dir (dir)
        {}

        //! @brief Destructor.
        virtual ~Ray () {}

        /**
         * @brief Get the position from which the ray starts.
         */
        rw::math::Vector3D<double>& pos () { return _pos; }
        
        /**
         * @brief Get the position from which the ray starts.
         */
        const rw::math::Vector3D<double>& pos () const { return _pos; }


        /**
         * @brief Get the direction in which the ray shoots.
         */
        rw::math::Vector3D<double>& dir () { return _dir; }

          /**
         * @brief Get the direction in which the ray shoots.
         */
        const rw::math::Vector3D<double>& dir () const { return _dir; }

        // inherited from Primitive
        //! @copydoc Primitive::createMesh
        TriMesh::Ptr createMesh (int resolution) const { return NULL; }

        //! @copydoc Primitive::getParameters
        rw::math::Q getParameters () const
        {
            return rw::math::Q (6, _pos[0], _pos[1], _pos[2], _dir[0], _dir[1], _dir[2]);
        }

        //! @copydoc Primitive::getType
        GeometryType getType () const { return LinePrim; }

#if !defined(SWIG)
        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<< (std::ostream& out, const Ray& ray)
        {
            return out << "Ray("
                       << "pos: " << ray._pos << ", dir: " << ray._dir << ")";
        };
#else
        TOSTRING (rw::geometry::Ray);
#endif

      private:
        rw::math::Vector3D<double> _pos, _dir;
    };

}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_RAY_HPP*/
