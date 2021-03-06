/*
 * PlanarSupportPoseGenerator.hpp
 *
 *  Created on: 19/08/2010
 *      Author: jimali
 */

#ifndef RWSIM_UTIL_PLANARSUPPORTPOSEGENERATOR_HPP_
#define RWSIM_UTIL_PLANARSUPPORTPOSEGENERATOR_HPP_

#include "SupportPose.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/geometry/Plane.hpp>

#include <vector>

namespace rw { namespace geometry {
    class ConvexHull3D;
}}    // namespace rw::geometry
namespace rw { namespace geometry {
    class Geometry;
}}    // namespace rw::geometry
namespace rw { namespace geometry {
    class TriMesh;
}}    // namespace rw::geometry
namespace rw { namespace kinematics {
    class Frame;
}}    // namespace rw::kinematics
namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics

namespace rwsim { namespace util {

    /**
     * @brief calculates the stable poses of an object when the support structure is
     * planar.
     *
     * This support pose generator implementation calculates the convex hull of an object
     * and projects the center of mass of the object onto all polygons of the convex hull.
     * If the projection is inside the polygon then the polygon is considered as a stable
     * support pose. Only projections that are within some threshold of the border of
     * a support polygon is used to further generate support poses.
     */
    class PlanarSupportPoseGenerator
    {
      public:
        /**
         * @brief constructor
         * @return
         */
        PlanarSupportPoseGenerator ();

        /**
         * @brief constructor - a convex hull generator can be supplied
         * @param hullGenerator
         * @return
         */
        PlanarSupportPoseGenerator (rw::core::Ptr< rw::geometry::ConvexHull3D > hullGenerator);

        /**
         * @brief calculates the hull and the support poses.
         * @param mesh
         */
        void analyze (const rw::geometry::TriMesh& mesh);

        void analyze (const std::vector< rw::core::Ptr< rw::geometry::Geometry > >& bodies,
                      rw::kinematics::Frame* ref, const rw::kinematics::State& state);

        void calculateDistribution (int i, std::vector< rw::math::Transform3D<> >& poses,
                                    std::vector< rw::math::Transform3D<> >& posesMises);
        /**
         * @brief gets the previously calculated support poses.
         * @return
         */
        std::vector< SupportPose > getSupportPoses ();

      private:
        void doAnalysis ();

        bool isInside (const rw::math::Vector3D<>& v, size_t i);

        void cleanup ()
        {
            _supportTriangles.clear ();
            _supportPoses.clear ();
            _supportPlanes.clear ();
        }

      private:
        rw::core::Ptr< rw::geometry::ConvexHull3D > _hullGenerator;
        std::vector< SupportPose > _supportPoses;
        std::vector< rw::geometry::Plane > _supportPlanes;
        std::vector< std::vector< rw::geometry::TriangleN1<double> > > _supportTriangles;
        rw::math::Vector3D<> _com;
    };

}}    // namespace rwsim::util

#endif /* PLANARSUPPORTPOSEGENERATOR_HPP_ */
