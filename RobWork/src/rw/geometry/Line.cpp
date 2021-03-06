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

#include "Line.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::geometry;
using namespace rw::math;

Line::Line () : Primitive (), _p1 (Vector3D<> ()), _p2 (Vector3D<>::z ())
{}

Line::Line (const rw::math::Q& initQ) : Primitive ()
{
    setParameters (initQ);
}

Line::Line (const rw::math::Vector3D<>& p1, const rw::math::Vector3D<>& p2) :
    Primitive (), _p1 (p1), _p2 (p2)
{}

Line::~Line ()
{}

void Line::setParameters (const rw::math::Q& q)
{
    if (q.size () != 6) {
        RW_THROW ("Size of parameter list must equal 6!");
    }

    _p1[0] = q (0);
    _p1[1] = q (1);
    _p1[2] = q (2);
    _p2[0] = q (3);
    _p2[1] = q (4);
    _p2[2] = q (5);
}

double Line::distance (const rw::math::Vector3D<>& point) const
{
    return fabs (cross (point - _p1, point - _p2).norm2 () / (_p2 - _p1).norm2 ());
}

double Line::distance (const Line& line) const
{
    Vector3D<> b = _p2 - _p1;
    Vector3D<> d = line._p2 - line._p1;

    double bdcross = cross (b, d).norm2 ();
    if (bdcross == 0.0)
        return 0.0;

    return fabs (dot (_p1 - line._p1, cross (b, d)) / bdcross);
}

rw::math::Vector3D<> Line::closestPoint (const rw::math::Vector3D<>& point) const
{
    Vector3D<> c0 = _p1 - point;
    Vector3D<> u  = _p2 - _p1;

    double t = -dot (c0, u) / dot (u, u);

    return point + c0 + t * u;
}

double Line::refit (const std::vector< rw::math::Vector3D<> >& data)
{
    return refit (data.cbegin (), data.cend ());
}

double Line::refit (const std::vector< rw::math::Vector3D<> >::const_iterator begin,
                    const std::vector< rw::math::Vector3D<> >::const_iterator end)
{
    /* check for data size - we need at least two points to make a line */
    if (std::distance (begin, end) < 2) {
        RW_THROW ("Data size must be 2 or more!");
    }

    /* calculate centroid */
    Vector3D<> centroid;
    for (std::vector< rw::math::Vector3D<> >::const_iterator it = begin; it != end; ++it) {
        centroid += (*it);
    }
    centroid /= static_cast< double > (std::distance (begin, end));

    /* perform singular value decomposition of data points */
    // create covariance matrix
    Eigen::MatrixXd covar (Eigen::MatrixXd::Zero (3, 3));
    for (std::vector< Vector3D<> >::const_iterator it = begin; it != end; it++) {
        const Vector3D<>& p = *it;
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                covar (i, j) += (p (i) - centroid (i)) * (p (j) - centroid (j));
            }
        }
    }

    // perform SVD
    Eigen::MatrixXd U;
    Eigen::VectorXd sigma;
    Eigen::MatrixXd V;
    LinearAlgebra::svd (covar, U, sigma, V);

    /* choose the first eigenvector */
    Vector3D<> dir (V.col (0));

    /* find point on the line closest to the origin */
    double t0 = -dot ((*begin), dir) / dot (dir, dir);
    _p1       = (*begin) + t0 * dir;
    _p2       = _p1 + normalize (dir);

    /* calculate fitting error */
    double error = 0.0;
    for (std::vector< rw::math::Vector3D<> >::const_iterator it = begin; it != end; ++it) {
        double d = distance (*it);
        error += d * d;
    }

    return error;
}

std::vector< Line > Line::makeGrid (int dim_x, int dim_y, double size_x, double size_y,
                                    const rw::math::Vector3D<>& xdir,
                                    const rw::math::Vector3D<>& ydir)
{
    // Rotation3D<> rot =< EAA<>(Vector3D<>::z(), normal).toRotation3D();
    rw::math::Vector3D<> xdir_n = normalize (xdir);
    rw::math::Vector3D<> ydir_n = normalize (ydir);
    std::vector< Line > lines;
    double halfsize_x = size_x * dim_x / 2.0;
    double halfsize_y = size_y * dim_y / 2.0;
    for (int dx = 0; dx <= dim_x; dx++) {
        Vector3D<> p1 = xdir_n * (dx * size_x - halfsize_x) + ydir_n * halfsize_y;
        Vector3D<> p2 = xdir_n * (dx * size_x - halfsize_x) + ydir_n * -halfsize_y;
        lines.push_back (Line (p2, p1));
    }
    for (int dy = 0; dy <= dim_y; dy++) {
        Vector3D<> p1 = xdir_n * halfsize_x + ydir_n * (dy * size_y - halfsize_y);
        Vector3D<> p2 = xdir_n * -halfsize_x + ydir_n * (dy * size_y - halfsize_y);
        lines.push_back (Line (p2, p1));
    }
    return lines;
}

rw::math::Metric< Line >::Ptr Line::makeMetric (double angToDistWeight)
{
    return rw::core::ownedPtr (new LineMetric (angToDistWeight));
}
