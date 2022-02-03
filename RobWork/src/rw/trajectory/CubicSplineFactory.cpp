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

#include "CubicSplineFactory.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/VectorND.hpp>
#include <rw/trajectory/CubicSplineInterpolator.hpp>
#include <rw/trajectory/SQUADInterpolator.hpp>

#include <Eigen/Sparse>
#include <math.h>

#if EIGEN_VERSION_AT_LEAST(3, 1, 0)
#include <Eigen/SparseCholesky>
#endif

using namespace rw::trajectory;

using namespace rw::math;
using namespace rw::core;

// ###########################################################################
// #                                Q Functions                              #
// ###########################################################################

InterpolatorTrajectory< Q >::Ptr CubicSplineFactory::makeNaturalSpline (QPath::Ptr qpath,
                                                                        double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < qpath->size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeNaturalSpline (*qpath, times);
}

InterpolatorTrajectory< rw::math::Q >::Ptr
CubicSplineFactory::makeNaturalSpline (TimedQPath::Ptr tqpath)
{
    QPath path;
    std::vector< double > times;
    for (const TimedQ& tq : *tqpath) {
        path.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }

    return makeNaturalSpline (path, times);
}

InterpolatorTrajectory< rw::math::Q >::Ptr
CubicSplineFactory::makeNaturalSpline (const QPath& qpath, const std::vector< double >& times)
{
    typedef float T;

    if (qpath.size () < 2)
        RW_THROW ("Path must be longer than 1!");
    if (qpath.size () != times.size ())
        RW_THROW ("Length of path and times need to be equal");

    size_t dim = (qpath)[0].size ();    // the number of dimensions of the points
    size_t N   = qpath.size () - 1;     // we have N+1 points, which yields N splines

    typedef Eigen::Matrix< T, Eigen::Dynamic, 1 > Vector;
    // typedef Eigen::Matrix<T, Eigen::Dynamic, 1, 1> Matrix;

    Vector B (N + 1);    // make room for boundary conditions

    Vector Y (N + 1);    // the points that the spline should intersect

    Vector a (dim * (N + 1)), b (dim * N), c (dim * N), d (dim * N);

    Vector H (N);    // duration from point i to i+1
    for (size_t i = 0; i < N; i++) {
        // T timeI0 = (T)((*tqpath)[i]).getTime();
        // T timeI1 = (T)((*tqpath)[i+1]).getTime();
        // H[i] = timeI1-timeI0;
        H[i] = (float) (times[i + 1] - times[i]);
    }

#if EIGEN_VERSION_AT_LEAST(3, 1, 0)
    Eigen::SparseMatrix< T > A ((int) N + 1, (int) N + 1);
    // D[0] = 2*H[0];
    A.insert (0, 0) = 2 * H[0];
    A.insert (0, 1) = H[0];
    A.insert (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        // D[i] = 2*(H[i-1]+H[i]);
        int ei                = (int) i;
        A.insert (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A.insert (ei, ei + 1) = H[i];
        A.insert (ei + 1, ei) = H[i];
    }
    // D[N] = 2*H[N-1];
    A.insert ((int) N, (int) N) = 2 * H[N - 1];

    Eigen::SimplicialLLT< Eigen::SparseMatrix< T > > solver;
    solver.compute (A);

#else
    Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > A =
        Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic >::Zero ((int) N + 1, (int) N + 1);
    A (0, 0) = 2 * H[0];
    A (0, 1) = H[0];
    A (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        // D[i] = 2*(H[i-1]+H[i]);
        int ei         = (int) i;
        A (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A (ei, ei + 1) = H[i];
        A (ei + 1, ei) = H[i];
    }
    // D[N] = 2*H[N-1];
    A ((int) N, (int) N) = 2 * H[N - 1];
    Eigen::LLT< Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > > solver;
    solver.compute (A);

#endif
    for (size_t j = 0; j < (size_t) dim; j++) {
        for (size_t i = 0; i < (size_t) N + 1; i++) {
            Y[i] = (T) (qpath[i])[j];
        }

        B[0] = (T) (3.0 * (Y[1] - Y[0]) / H[0]);
        for (size_t i = 1; i < N; i++) {
            B[i] = (T) (3.0 * ((Y[i + 1] - Y[i]) / H[i] - (Y[i] - Y[i - 1]) / H[i - 1]));
        }
        B[N] = (T) (-3.0 * (Y[N] - Y[N - 1]) / H[N - 1]);

        B = solver.solve (B);

        for (size_t i = 0; i < (size_t) N + 1; i++) {
            a[i * dim + j] = Y[i];
        }

        for (size_t i = 0; i < (size_t) N; i++) {
            c[j + i * dim] = B[i];
            b[j + i * dim] = (Y[i + 1] - Y[i]) / H[i] - H[i] * (B[i + 1] + 2 * B[i]) / (T) 3.0;
            d[j + i * dim] = (B[i + 1] - B[i]) / ((T) 3.0 * H[i]);    //   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory< Q >::Ptr traj = ownedPtr (new InterpolatorTrajectory< Q > (times[0]));

    Q ba (dim), bb (dim), bc (dim), bd (dim);
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < dim; j++) {
            ba[j] = a[j + i * dim];
            bb[j] = b[j + i * dim];
            bc[j] = c[j + i * dim];
            bd[j] = d[j + i * dim];
        }
        Interpolator< Q >* iptr = new CubicSplineInterpolator< Q > (ba, bb, bc, bd, H[i]);
        traj->add (ownedPtr (iptr));
    }

    return traj;
}

InterpolatorTrajectory< Q >::Ptr CubicSplineFactory::makeClampedSpline (QPath::Ptr qpath,
                                                                        const rw::math::Q& dqStart,
                                                                        const rw::math::Q& dqEnd,
                                                                        double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < qpath->size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeClampedSpline (*qpath, times, dqStart, dqEnd);
}

InterpolatorTrajectory< rw::math::Q >::Ptr
CubicSplineFactory::makeClampedSpline (TimedQPath::Ptr tqpath, const rw::math::Q& dqStart,
                                       const rw::math::Q& dqEnd)

{
    QPath path;
    std::vector< double > times;
    for (const TimedQ& tq : *tqpath) {
        path.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }

    return makeClampedSpline (path, times, dqStart, dqEnd);
}

// ###########################################################################
// #                             Template Functions                          #
// ###########################################################################

template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< T >& path, double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < path.size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeNaturalSpline (path, times);
}

// ### explicit template specifications
template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Vector3D<> >& path, double timeStep);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Transform3DVector<> >& path, double timeStep);

template InterpolatorTrajectory< Q >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Q >& path, double timeStep);

template InterpolatorTrajectory< Quaternion< double > >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Quaternion< double > >& path, double timeStep);

// ########### NEXT FUNCTION ###############

template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< T > >& path)
{
    Path< T > pathN;
    std::vector< double > times;
    for (const Timed< T >& tq : path) {
        pathN.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }

    return makeNaturalSpline (pathN, times);
}

// ### explicit template specifications
template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< Vector3D<> > >& path);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< Transform3DVector<> > >& path);

template InterpolatorTrajectory< Q >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< Q > >& path);

template InterpolatorTrajectory< Quaternion< double > >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< Quaternion< double > > >& path);

// ########### NEXT FUNCTION ###############

template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< T >& path, const std::vector< double >& times)
{
    if (path.size () < 2) {
        RW_THROW ("Path must be longer than 1!");
    }
    if (path.size () != times.size ()) {
        RW_THROW ("Length of path and times need to be equal");
    }

    size_t dim = path[0].size ();     // the number of dimensions of the points
    size_t N   = path.size () - 1;    // we have N+1 points, which yields N splines
    Eigen::VectorXd B (N + 1);        // make room for boundary conditions

    Eigen::VectorXd Y (N + 1);    // the points that the spline should intersect

    Eigen::VectorXd a (dim * (N + 1));
    Eigen::VectorXd b (dim * N);
    Eigen::VectorXd c (dim * N);
    Eigen::VectorXd d (dim * N);

    Eigen::VectorXd H (N);    // duration from point i to i+1
    for (size_t i = 0; i < N; i++) {
        H[i] = (double) (times[i + 1] - times[i]);
    }
#if EIGEN_VERSION_AT_LEAST(3, 1, 0)
    Eigen::SparseMatrix< double > A ((int) N + 1, (int) N + 1);
    A.insert (0, 0) = 2 * H[0];
    A.insert (0, 1) = H[0];
    A.insert (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        int ei                = (int) i;
        A.insert (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A.insert (ei, ei + 1) = H[i];
        A.insert (ei + 1, ei) = H[i];
    }
    // D[N] = 2*H[N-1];
    A.insert ((int) N, (int) N) = 2 * H[N - 1];

    Eigen::SimplicialLLT< Eigen::SparseMatrix< double > > solver;
    solver.compute (A);

#else
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > A =
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >::Zero ((int) N + 1, (int) N + 1);
    A (0, 0) = 2 * H[0];
    A (0, 1) = H[0];
    A (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        int ei         = (int) i;
        A (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A (ei, ei + 1) = H[i];
        A (ei + 1, ei) = H[i];
    }
    A ((int) N, (int) N) = 2 * H[N - 1];
    Eigen::LLT< Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > > solver;
    solver.compute (A);
#endif
    for (size_t j = 0; j < (size_t) dim; j++) {
        for (size_t i = 0; i < (size_t) N + 1; i++) {
            Y[i] = (double) (path[i])[j];
        }

        B[0] = (double) (3.0 * (Y[1] - Y[0]) / H[0]);
        for (size_t i = 1; i < N; i++) {
            B[i] = (double) (3.0 * ((Y[i + 1] - Y[i]) / H[i] - (Y[i] - Y[i - 1]) / H[i - 1]));
        }
        B[N] = (double) (-3.0 * (Y[N] - Y[N - 1]) / H[N - 1]);

        B = solver.solve (B);

        for (size_t i = 0; i < (size_t) N + 1; i++) {
            a[i * dim + j] = Y[i];
        }

        for (size_t i = 0; i < (size_t) N; i++) {
            c[j + i * dim] = B[i];
            b[j + i * dim] = (Y[i + 1] - Y[i]) / H[i] - H[i] * (B[i + 1] + 2 * B[i]) / (double) 3.0;
            d[j + i * dim] = (B[i + 1] - B[i]) / ((double) 3.0 * H[i]);    //   +B[i]+B[i+1];
        }
    }
    // ************** now create the actual trajectory from the calcualted parameters
    typename InterpolatorTrajectory< T >::Ptr traj =
        ownedPtr (new InterpolatorTrajectory< T > (times[0]));

    T ba, bb, bc, bd;
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < dim; j++) {
            ba[j] = a[j + i * dim];
            bb[j] = b[j + i * dim];
            bc[j] = c[j + i * dim];
            bd[j] = d[j + i * dim];
        }
        Interpolator< T >* iptr = new CubicSplineInterpolator< T > (ba, bb, bc, bd, H[i]);
        traj->add (ownedPtr (iptr));
    }
    return traj;
}

// ### explicit template specifications
template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Vector3D<> >& path,
                                       const std::vector< double >& times);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Transform3DVector<> >& path,
                                       const std::vector< double >& times);

template InterpolatorTrajectory< Quaternion<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Quaternion<> >& path,
                                       const std::vector< double >& times);

// ########### NEXT FUNCTION ###############
template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< T >& path, const T& dStart, const T& dEnd,
                                       double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < path.size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeClampedSpline (path, times, dStart, dEnd);
}
namespace rw { namespace trajectory {
template<>
typename InterpolatorTrajectory< Transform3D<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Transform3D<> >& path,
                                       const Transform3D<>& dStart, const Transform3D<>& dEnd,
                                       double timeStep)
{
    RW_THROW ("Clamped Cubic Spline not yet implemented for Transform3D");
    return NULL;
}
}}
template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Vector3D<> >& path, const Vector3D<>& dStart,
                                       const Vector3D<>& dEnd, double timeStep);

template InterpolatorTrajectory< Q >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Q >& path, const Q& dStart, const Q& dEnd,
                                       double timeStep);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Transform3DVector<> >& path,
                                       const Transform3DVector<>& dStart,
                                       const Transform3DVector<>& dEnd, double timeStep);

template InterpolatorTrajectory< Quaternion<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Quaternion<> >& path, const Quaternion<>& dStart,
                                       const Quaternion<>& dEnd, double timeStep);

// ########### NEXT FUNCTION ###############

template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Timed< T > >& tpath, const T& dStart,
                                       const T& dEnd)
{
    Path< T > path;
    std::vector< double > times;
    for (const Timed< T >& tq : tpath) {
        path.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }

    return makeClampedSpline (path, times, dStart, dEnd);
}

template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Timed< Vector3D<> > >& tpath,
                                       const Vector3D<>& dStart, const Vector3D<>& dEnd);

template InterpolatorTrajectory< Q >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Timed< Q > >& path, const Q& dStart,
                                       const Q& dEnd);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Timed< Transform3DVector<> > >& tpath,
                                       const Transform3DVector<>& dStart,
                                       const Transform3DVector<>& dEnd);

template InterpolatorTrajectory< Quaternion<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Timed< Quaternion<> > >& tpath,
                                       const Quaternion<>& dStart, const Quaternion<>& dEnd);

// ########### NEXT FUNCTION ###############
template< typename T >
typename InterpolatorTrajectory< T >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< T >& path, const std::vector< double >& times,
                                       const T& dStart, const T& dEnd)
{
    if (path.size () < 2)
        RW_THROW ("Path must be longer than 1!");

    if (path.size () != times.size ())
        RW_THROW ("Length of path and times need to match");

    size_t dim = path[0].size ();     // the number of dimensions of the points
    size_t N   = path.size () - 1;    // we have N+1 points, which yields N splines
    Eigen::VectorXd B (N + 1);        // make room for boundary conditions

    Eigen::VectorXd Y (N + 1);    // the points that the spline should intersect

    Eigen::VectorXd a (dim * (N + 1));
    Eigen::VectorXd b (dim * N);
    Eigen::VectorXd c (dim * N);
    Eigen::VectorXd d (dim * N);
    Eigen::VectorXd H (N);    // duration from point i to i+1

    for (size_t i = 0; i < N; i++) {
        H[i] = (float) (times[i + 1] - times[i]);
    }
#if EIGEN_VERSION_AT_LEAST(3, 1, 0)
    Eigen::SparseMatrix< double > A ((int) N + 1, (int) N + 1);

    A.insert (0, 0) = 2 * H[0];
    A.insert (0, 1) = H[0];
    A.insert (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        int ei                = (int) i;
        A.insert (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A.insert (ei + 1, ei) = H[i];
        A.insert (ei, ei + 1) = H[i];
    }
    A.insert ((int) N, (int) N) = 2 * H[N - 1];

    Eigen::SimplicialLLT< Eigen::SparseMatrix< double > > solver;
    solver.compute (A);

#else
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > A =
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >::Zero ((int) N + 1, (int) N + 1);

    A (0, 0) = 2 * H[0];
    A (0, 1) = H[0];
    A (1, 0) = H[0];
    for (size_t i = 1; i < N; i++) {
        int ei         = (int) i;
        A (ei, ei)     = 2 * (H[i - 1] + H[i]);
        A (ei + 1, ei) = H[i];
        A (ei, ei + 1) = H[i];
    }
    A ((int) N, (int) N) = 2 * H[N - 1];

    Eigen::LLT< Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > > solver;
    solver.compute (A);

#endif

    for (size_t j = 0; j < (size_t) dim; j++) {
        for (size_t i = 0; i < (size_t) N + 1; i++) {
            Y[i] = (double) (path[i])[j];
        }

        B[0] = (double) (3.0 * (Y[1] - Y[0]) / H[0] - 3 * dStart[j]);
        for (size_t i = 1; i < (std::size_t) B.size () - 1; i++) {
            B[i] = (double) (3.0 * ((Y[i + 1] - Y[i]) / H[i] - (Y[i] - Y[i - 1]) / H[i - 1]));
        }
        B[N] = (double) (3.0 * dEnd[j] - 3.0 * (Y[N] - Y[N - 1]) / H[N - 1]);

        B = solver.solve (B);

        for (size_t i = 0; i < N + 1; i++) {
            a[i * dim + j] = Y[i];
        }

        for (size_t i = 0; i < (size_t) N; i++) {
            c[j + i * dim] = B[i];
            b[j + i * dim] =
                (double) ((Y[i + 1] - Y[i]) / H[i] - H[i] * (B[i + 1] + 2 * B[i]) / 3.0);
            d[j + i * dim] = (double) ((B[i + 1] - B[i]) / (3.0 * H[i]));    //   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    typename InterpolatorTrajectory< T >::Ptr traj =
        ownedPtr (new InterpolatorTrajectory< T > (times[0]));

    T ba=path[0], bb=path[0], bc=path[0], bd=path[0];
    for (size_t i = 0; i < N; i++) {

        for (size_t j = 0; j < dim; j++) {
            ba[j] = a[j + i * dim];
            bb[j] = b[j + i * dim];
            bc[j] = c[j + i * dim];
            bd[j] = d[j + i * dim];
        }
        Interpolator< T >* iptr = new CubicSplineInterpolator< T > (ba, bb, bc, bd, H[i]);
        traj->add (ownedPtr (iptr));
    }

    return traj;
}

template InterpolatorTrajectory< Vector3D<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Vector3D<> >& path,
                                       const std::vector< double >& times, const Vector3D<>& dStart,
                                       const Vector3D<>& dEnd);

template InterpolatorTrajectory< Q >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Q >& path,
                                       const std::vector< double >& times, const Q& dStart,
                                       const Q& dEnd);

template InterpolatorTrajectory< Transform3DVector<> >::Ptr CubicSplineFactory::makeClampedSpline (
    const Path< Transform3DVector<> >& path, const std::vector< double >& times,
    const Transform3DVector<>& dStart, const Transform3DVector<>& dEnd);

template InterpolatorTrajectory< Quaternion<> >::Ptr
CubicSplineFactory::makeClampedSpline (const Path< Quaternion<> >& path,
                                       const std::vector< double >& times,
                                       const Quaternion<>& dStart, const Quaternion<>& dEnd);

// ###########################################################################
// #                          Transform3D Functions                          #
// ###########################################################################

InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Transform3D<> >& path, double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < path.size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeNaturalSpline (path, times);
}

InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< Timed< Transform3D<> > >& path)
{
    Path< Transform3D<> > pathN;
    std::vector< double > times;
    for (const Timed< Transform3D<> >& tq : path) {
        pathN.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }

    return makeNaturalSpline (pathN, times);
}

InterpolatorTrajectory< rw::math::Transform3DVector<> >::Ptr
CubicSplineFactory::makeNaturalSpline (const Path< rw::math::Transform3D<> >& path,
                                       const std::vector< double >& times)
{
    Path< Transform3DVector<> > NPath;
    for (const Transform3D<>& trans : path) {
        NPath.push_back (Transform3DVector<> (trans));
    }
    return makeNaturalSpline (NPath, times);
}

// ###########################################################################
// #                             SQUAD Functions                             #
// ###########################################################################

InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
CubicSplineFactory::makeSQUAD (const Path< rw::math::Quaternion<> >& path, double timeStep)
{
    std::vector< double > times;
    for (size_t i = 0; i < path.size (); i++) {
        times.push_back (i * timeStep);
    }
    return makeSQUAD (path, times);
}
InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
CubicSplineFactory::makeSQUAD (const Path< Timed< rw::math::Quaternion<> > >& tpath)
{
    Path< rw::math::Quaternion<> > pathN;
    std::vector< double > times;
    for (const Timed< rw::math::Quaternion<> >& tq : tpath) {
        pathN.push_back (tq.getValue ());
        times.push_back (tq.getTime ());
    }
    return makeSQUAD (pathN, times);
}

InterpolatorTrajectory< rw::math::Quaternion<> >::Ptr
CubicSplineFactory::makeSQUAD (const Path< rw::math::Quaternion<> >& path,
                               const std::vector< double >& times)
{
    InterpolatorTrajectory< Quaternion<> >::Ptr traj =
        ownedPtr (new InterpolatorTrajectory< Quaternion<> > (times[0]));
    size_t N = path.size () - 1;
    std::vector< Quaternion<> > s (path.size ());
    s[0] = path[0];
    for (size_t i = 1; i < s.size () - 1; i++) {
        const Quaternion<>& qn1 = path[i - 1];
        const Quaternion<>& q0  = path[i];
        const Quaternion<>& q1  = path[i + 1];

        s[i] = q0 * exp (-1 * (ln (q0.inverse () * q1) + ln (q0.inverse () * qn1)) * (1.0 / 4.0));
    }
    s.back () = path.back ();

    for (size_t i = 0; i < N - 1; i++) {
        double duration = times[i + 1] - times[i];
        Interpolator< Quaternion<> >* iptr =
            new SQUADInterpolator< double > (path[i], path[i + 1], s[i], s[i + 1], duration);
        traj->add (ownedPtr (iptr));
    }
    return traj;
}