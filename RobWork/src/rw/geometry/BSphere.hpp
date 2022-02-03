#ifndef RW_GEOMETRY_BSPHERE_HPP_
#define RW_GEOMETRY_BSPHERE_HPP_

#if !defined(SWIG)
#include "BV.hpp"

#include <rw/math/Constants.hpp>
#include <rw/math/Vector3D.hpp>
#endif
#include <rw/common/Traits.hpp>

namespace rw {
namespace geometry {
    class TriMesh;

    /**
     * @brief class representing an Bounding sphere
     */

    template< class T = double > class BSphere : public rw::geometry::BV< rw::geometry::BSphere< T > >
    {
      public:
        typedef T value_type;

        /**
         * @brief constructor using sphere center of (0, 0, 0)
         * @param radius [in] set the radius of the sphere
         */
        BSphere (T radius = 1.0) : _p3d (rw::math::Vector3D< T > (0, 0, 0)), _radius (radius) {}

        /**
         * @brief constructor setting both sphere center and radius
         * @param pos [in] the position of the center of the sphere
         * @param radius [in] set the radius of the sphere
         */
        BSphere (const rw::math::Vector3D< T >& pos, T radius = 1.0) : _p3d (pos), _radius (radius)
        {}

        /**
         * @brief Copy constroctor
         * @param bs [in] object to copy
         */
        BSphere (const rw::geometry::BSphere<T>& bs) : _p3d(bs.getPosition()),_radius(bs.getRadius()) {}

        /**
         * @brief get the position of the sphere center
         * @return a Vector3D with the center coordinates
         */
        inline const rw::math::Vector3D< T >& getPosition () const { return _p3d; }

        /**
         * @brief set the sphere center coordinate
         * @param p3d [in] the new center coordinates
         */
        inline void setPosition (const rw::math::Vector3D< T >& p3d) { _p3d = p3d; }

        /**
         * @brief get the sphere radius
         * @return sphere radius
         */
        inline const T& getRadius () const { return _radius; }

        /**
         * @brief get the sphere radius^2
         * @return sphere radius^2
         */
        inline const T getRadiusSqr () const { return _radius * _radius; }

        /**
         * @brief get the surface area
         * @return surface area
         */
        inline T calcArea () const { return (T) (4.0 * rw::math::Pi * _radius * _radius); }

        /**
         * @brief get the volume
         * @return volume
         */
        inline T calcVolume () const
        {
            return (T) (4.0 / 3.0 * rw::math::Pi * _radius * _radius * _radius);
        }
#if !defined(SWIG)
        /**
         * @brief Ouputs BSphere to stream
         * @param os [in/out] stream to use
         * @param sphere
         * @return the resulting stream
         */
        friend std::ostream& operator<< (std::ostream& os, const BSphere< T >& sphere)
        {
            return os << " BSphere{" << sphere._p3d << "," << sphere._radius << "}";
        }
#else
        TOSTRING (rw::geometry::BSphere<T>);
#endif
        /**
         * @brief fit a sphere in $O(n)$ to a triangle mesh using Principal Component Analysis (PCA)
         * where the eigen values of the vertices are used to compute the center of the sphere
         * using the vector with the maximum spread (largest eigenvalue).
         * @param tris [in] input mesh
         * @return bounding sphere
         */
        static BSphere< T > fitEigen (const rw::geometry::TriMesh& tris);

        // static BSphere<T> fitRitter(const rw::geometry::TriMesh& tris);
      private:
        rw::math::Vector3D< T > _p3d;
        T _radius;
    };

}    // namespace geometry
#if defined(SWIG)
SWIG_DECLARE_TEMPLATE (BVBSphere, rw::geometry::BV< rw::geometry::BSphere< double > >);
SWIG_DECLARE_TEMPLATE (BVBSphere_f, rw::geometry::BV< rw::geometry::BSphere< float > >);
#if SWIG_VERSION < 0x040000
SWIG_DECLARE_TEMPLATE (BSphere_d, rw::geometry::BSphere< double >);
ADD_DEFINITION(BSphere_d,BSphere)
#else 
SWIG_DECLARE_TEMPLATE (BSphere, rw::geometry::BSphere< double >);
#endif
SWIG_DECLARE_TEMPLATE (BSphere_f, rw::geometry::BSphere< float >);
#endif
template< typename T > struct Traits< geometry::BSphere< T > >
{
    typedef T value_type;
};

}    // namespace rw
#endif /*OBB_HPP_*/
