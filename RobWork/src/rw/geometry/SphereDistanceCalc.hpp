/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_SPHEREDISTANCECALC_HPP_
#define RW_PROXIMITY_SPHEREDISTANCECALC_HPP_

#if !defined(SWIG)
#include "BSphere.hpp"
#include "BVDistanceCalc.hpp"

#include <rw/geometry/OBB.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Vector3D.hpp>
#endif
namespace rw { namespace proximity {

    /**
     * @brief class for testing if two Oriented Bounding Boxes are overlapping
     */

    template< class T = double >
    class SphereDistanceCalc : public rw::proximity::BVDistanceCalc< SphereDistanceCalc< T >,
                                                                     rw::geometry::BSphere< T > >
    {
      public:
        typedef T value_type;

        //! @brief constructor
        SphereDistanceCalc (){};

        //! @brief destructor
        virtual ~SphereDistanceCalc (){};

        /**
         * @brief Calculates the distance between two bounding spheres.
         */
        inline T distance (const rw::geometry::BSphere< T >& a, const rw::geometry::BSphere< T >& b,
                           const rw::math::Vector3D< T >& aTb)
        {
            return aTb.norm2 () - (a.getRadius () + b.getRadius ());
        }

        /**
         * @brief Calculates the distance between two bounding spheres.
         */
        inline T distance (const rw::geometry::BSphere< T >& a, const rw::geometry::BSphere< T >& b)
        {
            return (a.getPosition () - b.getPosition ()).norm2 () -
                   (a.getRadius () + b.getRadius ());
        }

        /**
         * @brief calculates the squared distance between two bounding spheres.
         */
        inline T distanceSqr (const rw::geometry::BSphere< T >& a,
                              const rw::geometry::BSphere< T >& b,
                              const rw::math::Vector3D< T >& aTb)
        {
            return rw::math::MetricUtil::norm2Sqr (aTb) - (a.getRadiusSqr () + b.getRadiusSqr ());
        }
    };
#if defined(SWIG)
#define BVDistanceCalcSphereDistanceCalc_TYPE(type)                           \
    rw::proximity::BVDistanceCalc< rw::proximity::SphereDistanceCalc< type >, \
                                   rw::geometry::BSphere< type > >

#if SWIG_VERSION < 0x040000
    SWIG_DECLARE_TEMPLATE (BVDistanceCalcSphereDistanceCalc_d,
                           BVDistanceCalcSphereDistanceCalc_TYPE (double));
    SWIG_DECLARE_TEMPLATE (SphereDistanceCalc_d, rw::proximity::SphereDistanceCalc< double >);
    ADD_DEFINITION (BVDistanceCalcSphereDistanceCalc_d, BVDistanceCalcSphereDistanceCalc)
    ADD_DEFINITION (SphereDistanceCalc_d, SphereDistanceCalc)
#else
    SWIG_DECLARE_TEMPLATE (BVDistanceCalcSphereDistanceCalc,
                           BVDistanceCalcSphereDistanceCalc_TYPE (double));
    SWIG_DECLARE_TEMPLATE (SphereDistanceCalc, rw::proximity::SphereDistanceCalc< double >);
#endif
    SWIG_DECLARE_TEMPLATE (BVDistanceCalcSphereDistanceCalc_f,
                           BVDistanceCalcSphereDistanceCalc_TYPE (float));
    SWIG_DECLARE_TEMPLATE (SphereDistanceCalc_f, rw::proximity::SphereDistanceCalc< float >);


#endif
}}    // namespace rw::proximity

#endif
