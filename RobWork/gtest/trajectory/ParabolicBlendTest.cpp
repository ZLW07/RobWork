#include <rw/core/Exception.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>

#include <gtest/gtest.h>
#include <math.h>
#include <vector>

using namespace rw::trajectory;
using namespace rw::math;

TEST (ParabolicBlendTest, Constructor)
{
    LinearInterpolator< Vector2D<> > p1 (Vector2D<> (0, 0), Vector2D<> (1, 5), 2);
    LinearInterpolator< Vector2D<> > p2 (Vector2D<> (1, 5), Vector2D<> (1, 2), 0.1);
    LinearInterpolator< Vector2D<> > p3 (Vector2D<> (1, 5), Vector2D<> (2, 0), 2);
    LinearInterpolator< Vector2D<> > p4 (Vector2D<> (1, 5), Vector2D<> (2, 0), 0.1);
    LinearInterpolator< Vector2D<> > p5 (Vector2D<> (1, 4), Vector2D<> (2, 0), 2);
    LinearInterpolator< Vector2D<> > p6 (Vector2D<> (1, 5), Vector2D<> (1, 0), 2.5);

    EXPECT_THROW (ParabolicBlend< Vector2D< double > > b1 (p1, p2, 0.5), rw::core::Exception);
    ParabolicBlend< Vector2D< double > > b2 (p1, p3, 0.5);
    EXPECT_THROW (ParabolicBlend< Vector2D< double > > b3 (p1, p4, 0.5), rw::core::Exception);
    EXPECT_THROW (ParabolicBlend< Vector2D< double > > b4 (p1, p5, 0.5), rw::core::Exception);
    ParabolicBlend< Vector2D< double > > b5 (p1, p6, 0.6);

    EXPECT_DOUBLE_EQ (b2.tau1 (), b2.tau2 ());
    EXPECT_DOUBLE_EQ (b2.tau1 (), 0.5);
    EXPECT_DOUBLE_EQ (b2.duration (), 4.0);

    EXPECT_DOUBLE_EQ (b5.tau1 (), b5.tau2 ());
    EXPECT_DOUBLE_EQ (b5.tau1 (), 0.6);
    EXPECT_DOUBLE_EQ (b5.duration (), 4.5);
}

TEST (ParabolicBlendTest, PathVerification)
{
    InterpolatorTrajectory< Vector2D<> > traj;
    Vector2D< double > intersect (1, 5);
    Vector2D< double > end (2, 0);
    LinearInterpolator< Vector2D<> > p1 (Vector2D<> (0, 0), intersect, 2);
    LinearInterpolator< Vector2D<> > p2 (intersect, end, 1);
    ParabolicBlend< Vector2D<> > b1 (p1, p2, 0.25);

    traj.add (p1);
    traj.add (b1, p2);

    double min_dist_to_intersect   = (traj.x (0) - intersect).norm2 ();
    double max_dist_between_points = 0;

    Vector2D<> acc_vel = traj.dx (0);
    Vector2D<> acc_pos = traj.x (0);
    Vector2D<> vel_pos = traj.x (0);

    double dt = 0.001;
    for (double t = dt; t < traj.duration (); t += dt) {
        double d_to_i = (traj.x (t) - intersect).norm2 ();
        if (d_to_i < min_dist_to_intersect) {
            min_dist_to_intersect = d_to_i;
        }
        double t1   = t - dt;
        double dist = (traj.x (t) - traj.x (t1)).norm2 ();
        if (dist > max_dist_between_points) {
            max_dist_between_points = dist;
        }
        acc_pos += acc_vel * dt + 1 / 2.0 * traj.ddx (t) * dt * dt;
        acc_vel += traj.ddx (t1) * dt;
        vel_pos += traj.dx (t1) * dt;
    }
    EXPECT_LT (min_dist_to_intersect, 0.42);
    EXPECT_LT (max_dist_between_points, 0.0051);
    EXPECT_LT ((end - vel_pos).norm2 (), 0.004);
    EXPECT_LT ((end - acc_pos).norm2 (), 0.03);
}