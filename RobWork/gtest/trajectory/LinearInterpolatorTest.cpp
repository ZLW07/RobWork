
#include <rw/math/EAA.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <gtest/gtest.h>

using namespace rw::math;
using namespace rw::trajectory;

template< class T > bool close (T lhs, T rhs, double error = 1e-14)
{
    for (Eigen::Index x = 0; x < lhs.e ().rows (); x++) {
        for (Eigen::Index y = 0; y < lhs.e ().cols (); y++) {
            // std::cout << "point(" << x << "," << y << ") is " << lhs.e()(x,y)  << " = " <<
            // rhs.e()(x,y) << std::endl;
            if (std::abs (lhs.e () (x, y) - rhs.e () (x, y)) > error) {
                std::cout << "error: " << std::abs (lhs.e () (x, y) - rhs.e () (x, y)) << std::endl;
                std::cout << "point(" << x << "," << y << ") is " << lhs.e () (x, y) << " = "
                          << rhs.e () (x, y) << std::endl;
                return false;
            }
        }
    }
    return true;
}

Vector3Dd add (Vector3Dd position, Vector3Dd displacement, double dt = 1.0)
{
    return position + displacement*dt;
}

Rotation3Dd add (Rotation3Dd position, Rotation3Dd displacement, double dt = 1.0)
{
    return position * EAAd(displacement).scaleAngle(dt).toRotation3D();
}

Transform3Dd add (Transform3Dd position, Transform3Dd displacement, double dt = 1.0)
{
    Transform3Dd res;
    res.P () = position.P () + displacement.P () *dt;
    res.R () = position.R () * EAAd(displacement.R()).scaleAngle(dt).toRotation3D();
    return res;
}

template< typename T > class LinearInterpolatorTest : public testing::Test
{
  public:
    T start () { return T (); }
    T end ();
    double duration () { return 7.66; }
};

template<> Transform3Dd LinearInterpolatorTest< Transform3Dd >::end ()
{
    return rw::math::Math::ranTransform3D< double > (3.0);
}

template<> Rotation3Dd LinearInterpolatorTest< Rotation3Dd >::end ()
{
    return rw::math::Math::ranTransform3D< double > (3.0).R ();
}

template<> Vector3Dd LinearInterpolatorTest< Vector3Dd >::end ()
{
    return rw::math::Math::ranTransform3D< double > (3.0).P ();
}

using LineTypes = testing::Types< Vector3Dd, Rotation3Dd, Transform3Dd >;

TYPED_TEST_CASE (LinearInterpolatorTest, LineTypes);

TYPED_TEST (LinearInterpolatorTest, Constructor)
{
    TypeParam s = this->start ();
    TypeParam e = this->end ();
    double d    = this->duration ();
    LinearInterpolator< TypeParam > li (s, e, d);

    EXPECT_THROW (LinearInterpolator< TypeParam > (s, e, 0.0), rw::core::Exception);
    EXPECT_THROW (LinearInterpolator< TypeParam > (s, e, -1), rw::core::Exception);
    EXPECT_EQ (li.getStart (), s);
    EXPECT_TRUE (close (li.getEnd (), e));
    EXPECT_EQ (li.duration (), d);
}

TYPED_TEST (LinearInterpolatorTest, InterpolationSetup)
{
    TypeParam s = this->start ();
    TypeParam e = this->end ();
    double d    = this->duration ();
    LinearInterpolator< TypeParam > li (s, e, d);

    EXPECT_TRUE (close (li.x (0), s));
    EXPECT_TRUE (close (li.x (d), e));

    TypeParam sv = li.dx (0);
    for (double t = 0; t < li.duration (); t += li.duration () / 10) {
        EXPECT_EQ (li.dx (t), sv);
    }

    TypeParam sa = li.ddx (0);
    EXPECT_TRUE (close (sa, TypeParam ()));
    for (double t = 0; t < li.duration (); t += li.duration () / 10) {
        EXPECT_EQ (li.ddx (t), sa);
    }
}

TYPED_TEST (LinearInterpolatorTest, isLinear)
{
    TypeParam s = this->start ();
    TypeParam e = this->end ();
    double d    = 10.0;
    LinearInterpolator< TypeParam > li (s, e, d);

    TypeParam pos = li.x (0);
    double dt = 0.5;
    for (double t = 0; t < d; t += dt) {
        pos = add (pos, li.dx (t),dt);
    }
    EXPECT_TRUE (close (pos, li.x (d),0.0005));
}