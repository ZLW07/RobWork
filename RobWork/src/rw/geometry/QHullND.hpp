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

#ifndef RW_GEOMETRY_QHULLND_HPP_
#define RW_GEOMETRY_QHULLND_HPP_

#if !defined(SWIG)
#include "ConvexHullND.hpp"

#include <rw/core/macros.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/math/VectorND.hpp>

#include <float.h>
#include <vector>
#endif 

namespace rw { namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file QHullND.hpp
     */

    namespace qhull {

        /**
         * @brief calclates the convex hull of a set of vertices \b coords each with dimension  \b
         * dim
         *
         * @param dim [in] nr of dimensions in each vertice
         * @param coords [in] array of vertices
         * @param nrCoords [in] the number of vertices
         * @param vertIdxs
         * @param faceIdxs
         * @param faceNormals
         * @param faceOffsets
         */
        void build (size_t dim, double* coords, size_t nrCoords, std::vector< int >& vertIdxs,
                    std::vector< int >& faceIdxs, std::vector< double >& faceNormals,
                    std::vector< double >& faceOffsets);

        /**
         * @brief calclates the convex hull of a set of vertices \b coords each with dimension  \b
         * dim
         *
         * @param coords [in] array of vertices
         * @param vertIdxs
         * @param faceIdxs
         * @param faceNormals
         * @param faceOffsets
         */
        void build (std::vector<std::vector<double>> coords, std::vector< int >& vertIdxs,
                    std::vector< int >& faceIdxs, std::vector< double >& faceNormals,
                    std::vector< double >& faceOffsets);
    }    // namespace qhull

    /**
     * @brief calculates the convex hull of a set of 3d points.
     *
     * The GirftWrap convex hull algorithm is used, hence the
     * class name.
     *
     * @note It is important that there are not multiple vertices at the same coordinates.
     * Filter these away before using this convex hull calculation.
     */
    template< std::size_t N > class QHullND : public ConvexHullND< N >
    {
      public:
        //! @brief constructor
        QHullND () {}

        //! @brief destructor
        virtual ~QHullND () {}

        //! @copydoc ConvexHullND::rebuild
        void rebuild (const std::vector< rw::math::VectorND< N > >& vertices)
        {
            using namespace rw::math;
            // convert the vertice array to an array of double
            double* vertArray = new double[vertices.size () * N];
            // copy all data into the vertArray
            for (size_t i = 0; i < vertices.size (); i++) {
                const VectorND< N >& vnd = vertices[i];
                for (size_t j = 0; j < N; j++)
                    vertArray[i * N + j] = vnd[j];
            }
            // build the hull
            qhull::build (N,
                          vertArray,
                          vertices.size (),
                          _vertiIdxs,
                          _faceIdxs,
                          _faceNormalsTmp,
                          _faceOffsets);
            delete[] vertArray;

            std::vector< int > vertIdxMap (vertices.size ());
            _hullVertices.resize (_vertiIdxs.size ());
            for (size_t i = 0; i < _vertiIdxs.size (); i++) {
                _hullVertices[i]          = vertices[_vertiIdxs[i]];
                vertIdxMap[_vertiIdxs[i]] = (int) i;
            }
            for (size_t i = 0; i < _faceIdxs.size (); i++) {
                int tmp      = _faceIdxs[i];
                _faceIdxs[i] = vertIdxMap[tmp];
            }
            _faceOffsets.resize (_faceIdxs.size () / N);
            _faceNormals.resize (_faceIdxs.size () / N);
            for (size_t i = 0; i < _faceIdxs.size () / N; i++) {
                for (size_t j = 0; j < N; j++)
                    _faceNormals[i][j] = _faceNormalsTmp[i * N + j];
            }
        }

        /**
         * @brief Check if a point is inside the hull.
         * @param vertex [in] the vertex to check.
         * @return true if \b vertex lies on hull or inside, false otherwise.
         */
        virtual bool isInside (const rw::math::VectorND< N >& vertex)
        {
            using namespace rw::math;
            // const static double EPSILON = 0.0000001;
            if (_faceIdxs.size () == 0) {
                // std::cout << "No Tris" << std::endl;
                return 0;
            }

            double minDist = DBL_MAX;
            for (size_t i = 0; i < _faceIdxs.size () / N; i++) {
                RW_ASSERT (_faceIdxs.size () > i * N);
                // RW_ASSERT(faceVerticeIdx<(int)vertices.size());
                RW_ASSERT (_faceIdxs[i * N] < (int) _hullVertices.size ());
                RW_ASSERT (i < _faceNormals.size ());
                double dist = _faceOffsets[i] + dot (vertex, _faceNormals[i]);
                // dist will be negative if point is inside, and positive if point is outside
                minDist = std::min (-dist, minDist);
                if (minDist < 0)
                    return false;
            }

            return minDist >= 0;
        }

        /**
         * @brief Calculates a distance to the hull. 
         * The distance to the \b vertex is greater or equal to the result
         * @param vertex [in] vertex to calculate minimum distance for.
         * @return a distance <= the actual distancte to the hull if \b vertex is outside, otherwise zero is
         * returned.
         */
        virtual double getMinDistOutside (const rw::math::VectorND< N >& vertex)
        {
            if (_faceIdxs.size () == 0) {
                return 0;
            }
            double minDist = DBL_MAX;
            bool isOutside = false;
            for (size_t i = 0; i < _faceIdxs.size () / N; i++) {
                RW_ASSERT (_faceIdxs.size () > i * N);
                RW_ASSERT (i < _faceNormals.size ());
                // dist will be negative if point is inside, and positive if point is outside
                const double dist = _faceOffsets[i] + rw::math::dot (vertex, _faceNormals[i]);
                if (dist > 0) {
                    isOutside = true;
                    minDist   = std::min (dist, minDist);
                }
            }
            return (isOutside ? minDist : 0);
        }

        /**
         * @brief Calculates the minimum distance to the hull.
         * @param vertex [in] vertex to calculate distance for.
         * @return the minimum distance if \b vertex is inside the hull (as a positive value).
         * Currently, the maximum distance is returned if \b vertex is outside the hull (as a
         * negative value). This is subject to change, so do not rely on this behaviour.
         */
        virtual double getMinDistInside (const rw::math::VectorND< N >& vertex)
        {
            using namespace rw::math;
            if (_faceIdxs.size () == 0) {
                return 0;
            }
            double minDist = DBL_MAX;
            for (size_t i = 0; i < _faceIdxs.size () / N; i++) {
                RW_ASSERT (_faceIdxs.size () > i * N);
                RW_ASSERT (i < _faceNormals.size ());
                double dist = _faceOffsets[i] + dot (vertex, _faceNormals[i]);
                // dist will be negative if point is inside, and positive if point is outside
                minDist = std::min (-dist, minDist);
            }

            return (minDist>0?minDist:0);
        }

        //! @copydoc ConvexHullND::getAvgDistInside
        virtual double getAvgDistInside (const rw::math::VectorND< N >& vertex)
        {
            using namespace rw::math;

            // check if we have any faces
            RW_ASSERT (_faceNormals.size () > 0);

            // loop over all 'faces' and calculate their areas
            int nOfFaces = (int) (_faceIdxs.size () / N);

            double totalVolume = 0.0;
            double avgDist     = 0.0;
            // std::cout << "N of faces= " << nOfFaces << std::endl;
            for (size_t i = 0; (int) i < nOfFaces; ++i) {
                RW_ASSERT (i < _faceNormals.size ());

                double dist = _faceOffsets[i] + dot (vertex, _faceNormals[i]);

                // calculate weight (by face volume)
                std::vector< VectorND< N > > v;
                for (int j = 0; j < (int) N; ++j) {
                    v.push_back (_hullVertices[_faceIdxs[i * N + j]]);
                }
                double volume = GeometryUtil::simplexVolume (v);

                totalVolume += volume;
                avgDist += -dist * volume;
            }

            avgDist /= totalVolume;

            return avgDist;
        }

        //! @copydoc ConvexHullND::getCentroid
        virtual rw::math::VectorND< N > getCentroid ()
        {
            using namespace rw::math;

            // check if we have any faces
            RW_ASSERT (_faceNormals.size () > 0);

            // loop over all 'faces' and calculate their areas and centroids
            // std::vector<double> areas;
            // std::vector<VectorND<N> > centroids;
            int nOfFaces           = (int) (_faceIdxs.size () / N);
            VectorND< N > centroid = VectorND< N >::zero ();
            ;
            double totalVolume = 0.0;
            // std::cout << "N of faces= " << nOfFaces << std::endl;
            for (size_t i = 0; (int) i < nOfFaces; ++i) {
                RW_ASSERT (_faceIdxs.size () > i * N);
                RW_ASSERT (i < _faceNormals.size ());

                // calculate face centroid
                VectorND< N > c = VectorND< N >::zero ();
                for (int j = 0; j < (int) N; ++j) {
                    c += (1.0 / N) * _hullVertices[_faceIdxs[i * N + j]];
                    // std::cout << "v: " << _hullVertices[_faceIdxs[i*N + j]] << std::endl;
                }
                // calculate weight (by face volume)
                std::vector< VectorND< N > > v;
                for (int j = 0; j < (int) N; ++j) {
                    v.push_back (_hullVertices[_faceIdxs[i * N + j]]);
                }
                double volume = GeometryUtil::simplexVolume (v);
                // std::cout << "C= " << c << std::endl;
                // std::cout << "V= " << volume << std::endl;

                // update hull centroid
                centroid += volume * c;
                totalVolume += volume;
            }

            centroid /= totalVolume;

            return centroid;
        }

        //! @copydoc ConvexHullND::getClosestPoint
        virtual rw::math::VectorND< N > getClosestPoint (const rw::math::VectorND< N >& vertex)
        {
            using namespace rw::math;

            if (_faceIdxs.size () == 0) {
                return VectorND< N > ();
            }

            double min_dist = DBL_MAX;
            VectorND< N > closest_point;

            for (size_t i = 0; i < _faceIdxs.size () / N; i++) {
                RW_ASSERT (_faceIdxs.size () > i * N);
                RW_ASSERT (i < _faceNormals.size ());

                double dist = _faceOffsets[i] + dot (vertex, _faceNormals[i]);

                // dist will be negative if point is inside, and positive if point is outside
                dist = -dist;

                if (dist < min_dist) {
                    min_dist      = dist;
                    closest_point = fabs (dist) * _faceNormals[i];
                }
            }

            return closest_point;
        }

        /**
         * @brief Calculates the volume of the hull.
         * @return The volume of the QHull.
         */
        virtual double getVolume () const
        {
            // check if we have any faces
            RW_ASSERT (_faceNormals.size () > 0);

            // loop over all 'faces' and calculate their areas
            const size_t nOfFaces = _faceIdxs.size () / N;
            RW_ASSERT (nOfFaces <= _faceNormals.size ());

            double totalVolume = 0.0;
            for (size_t i = 0; i < nOfFaces; ++i) {
                std::vector< rw::math::VectorND< N > > v;
                for (size_t j = 0; j < N; j++) {
                    v.push_back (_hullVertices[_faceIdxs[i * N + j]]);
                }
                const double volume = rw::geometry::GeometryUtil::actualSimplexVolume (v);
                if (_faceOffsets.at (i) <= 0.0) {
                    totalVolume += volume;
                }
                else {
                    totalVolume -= volume;
                }
            }
            return totalVolume;
        }

        /**
         * @brief Returns the list hull vertices
         * @return Reference to the internal vector with hull vertices
         */
        const std::vector< rw::math::VectorND< N > >& getHullVertices () { return _hullVertices; }

        /**
         * @brief Returns the list with face indices
         * @return Reference to the internal vector with face normals
         */
        const std::vector< int >& getFaceIndices () { return _faceIdxs; }

        /**
         * @brief Returns the face normals
         * @return Reference to the internal vector with face normals
         */
        const std::vector< rw::math::VectorND< N > >& getFaceNormals () { return _faceNormals; }

        /**
         * @brief Returns the offsets (distance from origin to surface in direction of the normal)
         * of the faces
         * @return Reference to the internal list face offsets
         */
        const std::vector< double >& getFaceOffsets () { return _faceOffsets; }

      private:
        std::vector< rw::math::VectorND< N > > _hullVertices, _faceNormals;
        std::vector< double > _faceOffsets;
        std::vector< int > _vertiIdxs, _faceIdxs;
        std::vector< double > _faceNormalsTmp;
    };
    //! @}
}}     // namespace rw::geometry
#endif /* GIFTWRAPHULL_HPP_ */
