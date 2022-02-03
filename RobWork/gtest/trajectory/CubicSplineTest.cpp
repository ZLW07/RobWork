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

#include <rw/core/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Transform3DVector.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>

#include <gtest/gtest.h>

using namespace rw::core;
using namespace rw::math;
using namespace rw::trajectory;

#define TRANSFORM3D_TYPE 1

bool close (Quaternion<> q1, Quaternion<> q2, std::vector< double > diff)
{
    bool res = fabs ((q1).getQx () - (q2).getQx ()) < diff[0] &&
               fabs ((q1).getQy () - (q2).getQy ()) < diff[1] &&
               fabs ((q1).getQz () - (q2).getQz ()) < diff[2] &&
               fabs ((q1).getQw () - (q2).getQw ()) < diff[3];
    if (!res) {
        std::cout << std::endl;
        std::cout << "diff1: " << fabs ((q1).getQx () - (q2).getQx ()) << std::endl;
        std::cout << "diff1: " << fabs ((q1).getQy () - (q2).getQy ()) << std::endl;
        std::cout << "diff1: " << fabs ((q1).getQz () - (q2).getQz ()) << std::endl;
        std::cout << "diff1: " << fabs ((q1).getQw () - (q2).getQw ()) << std::endl;
    }
    return res;
}

template< typename T > class CubicSplineTypeTest : public testing::Test
{
  public:
    T getPoint (size_t i);
    T start ();
    T end ();
    int type ();

    const std::vector< EAA<> > angles = {
        EAA<> (22, 0, 0), EAA<> (0, -1, 0), EAA<> (0, 0, 1), EAA<> (0, 2, 0)};
    const std::vector< Vector3D<> > pos = {Vector3D<> (0, 0.5, 0),
                                           Vector3D<> (0, 0, 0.5),
                                           Vector3D<> (0.5, 0, 0),
                                           Vector3D<> (0, 0, 0)};
};

// ############### getPoints ########################
template<> Q CubicSplineTypeTest< Q >::getPoint (size_t i)
{
    Q q = Q::zero (7);
    switch (i) {
        case 0: q (0) = 1; return q;
        case 1: q (0) = 8; return q;
        case 2: q (0) = -1; return q;
        case 3: q (0) = 4; return q;
        default: return q;
    }
}
template<> Transform3D<> CubicSplineTypeTest< Transform3D<> >::getPoint (size_t i)
{
    return Transform3D<> (pos[i], angles[i]);
}
template<> Transform3DVector<> CubicSplineTypeTest< Transform3DVector<> >::getPoint (size_t i)
{
    return Transform3DVector<> (pos[i], angles[i]);
}
template<> Vector3D<> CubicSplineTypeTest< Vector3D<> >::getPoint (size_t i)
{
    return pos[i];
}
template<> Quaternion<> CubicSplineTypeTest< Quaternion<> >::getPoint (size_t i)
{
    return Quaternion<> (angles[i]);
}

// ############### start ########################
template<> Q CubicSplineTypeTest< Q >::start ()
{
    return (getPoint (0) - getPoint (1) / 2);
}
template<> Transform3D<> CubicSplineTypeTest< Transform3D<> >::start ()
{
    return Transform3D<> ((getPoint (0).P () - getPoint (1).P () / 2),
                          (getPoint (0).R () * getPoint (1).R ().inverse (true)));
}
template<> Transform3DVector<> CubicSplineTypeTest< Transform3DVector<> >::start ()
{
    return (getPoint (0) - getPoint (1) / 2);
}
template<> Vector3D<> CubicSplineTypeTest< Vector3D<> >::start ()
{
    return (getPoint (0) - getPoint (1) / 2);
}
template<> Quaternion<> CubicSplineTypeTest< Quaternion<> >::start ()
{
    return (getPoint (0) - getPoint (1)).elemDivide (2);
}

// ############### end ########################
template<> Q CubicSplineTypeTest< Q >::end ()
{
    return (getPoint (2) - getPoint (3) / 2);
}
template<> Transform3D<> CubicSplineTypeTest< Transform3D<> >::end ()
{
    return Transform3D<> ((getPoint (2).P () - getPoint (3).P () / 2),
                          (getPoint (2).R () * getPoint (3).R ().inverse (true)));
}
template<> Transform3DVector<> CubicSplineTypeTest< Transform3DVector<> >::end ()
{
    return (getPoint (2) - getPoint (3) / 2);
}
template<> Vector3D<> CubicSplineTypeTest< Vector3D<> >::end ()
{
    return (getPoint (2) - getPoint (3) / 2);
}
template<> Quaternion<> CubicSplineTypeTest< Quaternion<> >::end ()
{
    return (getPoint (2) - getPoint (3)).elemDivide (2);
}

// ############## type #########################
template<> int CubicSplineTypeTest< Q >::type ()
{
    return 0;
}
template<> int CubicSplineTypeTest< Transform3D<> >::type ()
{
    return 1;
}
template<> int CubicSplineTypeTest< Transform3DVector<> >::type ()
{
    return 2;
}
template<> int CubicSplineTypeTest< Vector3D<> >::type ()
{
    return 3;
}
template<> int CubicSplineTypeTest< Quaternion<> >::type ()
{
    return 4;
}

// ############### isContinues ####################

template< class T > bool isContinues (InterpolatorTrajectory< T >& traj);

template<> bool isContinues (InterpolatorTrajectory< Q >& traj)
{
    double start = traj.startTime ();
    Q acc_vel    = traj.dx (start);
    Q acc_pos    = traj.x (start);
    Q vel_pos    = traj.x (start);

    Q last_x   = traj.x (start);
    Q last_dx  = traj.dx (start);
    Q last_ddx = traj.ddx (start);

    double dt = 0.0005;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        double t1 = t - dt;
        acc_pos += acc_vel * dt + 1 / 2.0 * traj.ddx (t) * dt * dt;
        acc_vel += traj.ddx (t1) * dt;
        vel_pos += traj.dx (t1) * dt;
        EXPECT_LT (MetricUtil::dist2 (acc_pos, traj.x (t)), 0.041);
        EXPECT_LT (MetricUtil::dist2 (vel_pos, traj.x (t)), 0.004);

        EXPECT_LT (MetricUtil::dist2 (last_x, traj.x (t)), 0.007);
        EXPECT_LT (MetricUtil::dist2 (last_dx, traj.dx (t)), 0.03);
        EXPECT_LT (MetricUtil::dist2 (last_ddx, traj.ddx (t)), 0.06);

        last_x   = traj.x (t);
        last_dx  = traj.dx (t);
        last_ddx = traj.ddx (t);
    }

    return true;
}
template<> bool isContinues (InterpolatorTrajectory< Transform3D<> >& traj)
{
    /*// Position Varification
    double start       = traj.startTime ();
    Vector3D<> acc_vel = traj.dx (start).P ();
    Vector3D<> acc_pos = traj.x (start).P ();
    Vector3D<> vel_pos = traj.x (start).P ();

    Vector3D<> last_x   = traj.x (start).P ();
    Vector3D<> last_dx  = traj.dx (start).P ();
    Vector3D<> last_ddx = traj.ddx (start).P ();

    double dt = 0.001;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        double t1 = t - dt;

        acc_pos += acc_vel * dt + 1 / 2.0 * traj.ddx (t).P () * dt * dt;
        acc_vel += traj.ddx (t1).P () * dt;
        vel_pos += traj.dx (t1).P () * dt;

        EXPECT_LT (MetricUtil::dist2 (acc_pos, traj.x (t).P ()), 0.01);
        EXPECT_LT (MetricUtil::dist2 (vel_pos, traj.x (t).P ()), 0.005);

        EXPECT_LT (MetricUtil::dist2 (last_x, traj.x (t).P ()), 0.003);
        EXPECT_LT (MetricUtil::dist2 (last_dx, traj.dx (t).P ()), 0.006);
        EXPECT_LT (MetricUtil::dist2 (last_ddx, traj.ddx (t).P ()), 0.01);

        last_x   = traj.x (t).P ();
        last_dx  = traj.dx (t).P ();
        last_ddx = traj.ddx (t).P ();
    }
*/
    double start       = traj.startTime ();

    Vector3D<> last_px = traj.x (start).P ();

    double dt = 0.001;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        EXPECT_LT (MetricUtil::dist2 (last_px , traj.x (t).P ()), 0.003);

        last_px = traj.x (t).P ();
    }

    Quaternion<> last_rx = traj.x (start).R ();

    dt = 0.0005;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        EXPECT_TRUE (
            close (last_rx, Quaternion<> (traj.x (t).R ()), {0.002, 0.0017, 0.001, 0.003}));

        last_rx = traj.x (t).R ();
    }

    return true;
}

template<> bool isContinues (InterpolatorTrajectory< Transform3DVector<> >& traj)
{
    double start       = traj.startTime ();

    Vector3D<> last_px = traj.x (start).toVector3D ();

    double dt = 0.001;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        EXPECT_LT (MetricUtil::dist2 (last_px , traj.x (t).toVector3D ()), 0.003);

        last_px = traj.x (t).toVector3D ();
    }
    /*Vector3D<> acc_vel = traj.dx (start).toVector3D ();
    Vector3D<> acc_pos = traj.x (start).toVector3D ();
    Vector3D<> vel_pos = traj.x (start).toVector3D ();

    Vector3D<> last_x   = traj.x (start).toVector3D ();
    Vector3D<> last_dx  = traj.dx (start).toVector3D ();
    Vector3D<> last_ddx = traj.ddx (start).toVector3D ();

    double dt = 0.001;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        double t1 = t - dt;

        acc_pos += acc_vel * dt + 1 / 2.0 * traj.ddx (t).toVector3D () * dt * dt;
        acc_vel += traj.ddx (t1).toVector3D () * dt;
        vel_pos += traj.dx (t1).toVector3D () * dt;

        EXPECT_LT (MetricUtil::dist2 (acc_pos, traj.x (t).toVector3D ()), 0.01);
        EXPECT_LT (MetricUtil::dist2 (vel_pos, traj.x (t).toVector3D ()), 0.005);

        EXPECT_LT (MetricUtil::dist2 (last_x, traj.x (t).toVector3D ()), 0.003);
        EXPECT_LT (MetricUtil::dist2 (last_dx, traj.dx (t).toVector3D ()), 0.006);
        EXPECT_LT (MetricUtil::dist2 (last_ddx, traj.ddx (t).toVector3D ()), 0.01);

        last_x   = traj.x (t).toVector3D ();
        last_dx  = traj.dx (t).toVector3D ();
        last_ddx = traj.ddx (t).toVector3D ();
    }*/

    Quaternion<> last_rx = traj.x (start).toQuaternion ();

    dt = 0.0005;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        EXPECT_TRUE (close (last_rx, traj.x (t).toQuaternion (), {0.002, 0.0017, 0.001, 0.003}));

        last_rx = traj.x (t).toQuaternion ();
    }

    return true;
}
template<> bool isContinues (InterpolatorTrajectory< Vector3D<> >& traj)
{
    double start       = traj.startTime ();
    Vector3D<> acc_vel = traj.dx (start);
    Vector3D<> acc_pos = traj.x (start);
    Vector3D<> vel_pos = traj.x (start);

    Vector3D<> last_x   = traj.x (start);
    Vector3D<> last_dx  = traj.dx (start);
    Vector3D<> last_ddx = traj.ddx (start);

    double dt = 0.001;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        double t1 = t - dt;

        acc_pos += acc_vel * dt + 1 / 2.0 * traj.ddx (t) * dt * dt;
        acc_vel += traj.ddx (t1) * dt;
        vel_pos += traj.dx (t1) * dt;

        EXPECT_LT (MetricUtil::dist2 (acc_pos, traj.x (t)), 0.01);
        EXPECT_LT (MetricUtil::dist2 (vel_pos, traj.x (t)), 0.005);

        EXPECT_LT (MetricUtil::dist2 (last_x, traj.x (t)), 0.003);
        EXPECT_LT (MetricUtil::dist2 (last_dx, traj.dx (t)), 0.006);
        EXPECT_LT (MetricUtil::dist2 (last_ddx, traj.ddx (t)), 0.01);

        last_x   = traj.x (t);
        last_dx  = traj.dx (t);
        last_ddx = traj.ddx (t);
    }

    return true;
}
template<> bool isContinues (InterpolatorTrajectory< Quaternion<> >& traj)
{
    double start        = traj.startTime ();
    Quaternion<> last_x = traj.x (start);

    double dt = 0.0005;
    for (double t = start + dt; t < traj.endTime (); t += dt) {
        EXPECT_TRUE (close (last_x, traj.x (t), {0.002, 0.0017, 0.001, 0.003}));

        last_x = traj.x (t);
    }

    return true;
}

using SplineTypes =
    testing::Types< Q, Transform3D<>, Quaternion<>, Vector3D<>, Transform3DVector<> >;
TYPED_TEST_CASE (CubicSplineTypeTest, SplineTypes);

TYPED_TEST (CubicSplineTypeTest, PathToShort)
{
    // check throw on empty QPath
    Path< TypeParam > path;
    EXPECT_THROW (CubicSplineFactory::makeNaturalSpline (path), rw::core::Exception);
    path.push_back (TypeParam ());
    EXPECT_THROW (CubicSplineFactory::makeNaturalSpline (path), rw::core::Exception);
}

TEST (CubicSplineTest, PathToShort_QPtr)
{
    // check throw on empty QPath
    QPath::Ptr path = ownedPtr (new QPath ());
    EXPECT_THROW (CubicSplineFactory::makeNaturalSpline (path), rw::core::Exception);
    path->push_back (Q (3));
    EXPECT_THROW (CubicSplineFactory::makeNaturalSpline (path), rw::core::Exception);
}

TYPED_TEST (CubicSplineTypeTest, PathTest)
{
    Path< TypeParam > path;
    path.push_back (this->getPoint (0));
    path.push_back (this->getPoint (1));
    path.push_back (this->getPoint (2));
    path.push_back (this->getPoint (3));

    auto traj = CubicSplineFactory::makeNaturalSpline (path);
    EXPECT_TRUE (traj != NULL);
    EXPECT_NEAR (traj->duration (), (double) path.size () - 1, 0.0001);

    EXPECT_TRUE (isContinues (*traj));

    if (this->type () != TRANSFORM3D_TYPE) {
        auto traj2 = CubicSplineFactory::makeClampedSpline (path, this->start (), this->end ());

        EXPECT_TRUE (traj2 != NULL);
        EXPECT_NEAR (traj2->duration (), (double) path.size () - 1, 0.0001);
        EXPECT_TRUE (isContinues (*traj2));
    }
}

TEST (CubicSplineTest, PathTest_QPtr)
{
    QPath::Ptr path = ownedPtr (new QPath ());
    Q q             = Q::zero (7);
    q (0)           = 1;
    path->push_back (q);
    q (0) = 8;
    path->push_back (q);
    q (0) = -1;
    path->push_back (q);
    q (0) = 4;
    path->push_back (q);

    auto traj = CubicSplineFactory::makeNaturalSpline (path);
    EXPECT_TRUE (traj != NULL);
    EXPECT_NEAR (traj->duration (), (double) path->size () - 1, 0.0001);

    EXPECT_TRUE (isContinues (*traj));

    Q start = Q::zero (7), end = Q::zero (7);
    start[0] = 1;
    end[0]   = -1;
    traj     = CubicSplineFactory::makeClampedSpline (path, start, end);

    EXPECT_TRUE (traj != NULL);
    EXPECT_NEAR (traj->duration (), (double) path->size () - 1, 0.0001);
    EXPECT_TRUE (isContinues (*traj));
}
