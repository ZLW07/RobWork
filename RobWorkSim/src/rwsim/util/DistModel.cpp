#include "DistModel.hpp"

#include <rw/core/macros.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rwsim::util;

double DistModel::refit (std::vector< rw::math::Vector3D<> >& data)
{
    if (data.size () == 0)
        RW_THROW ("Data size must be 1 or more!");

    _invalid = false;

    using namespace rw::math;

    Vector3D<> centroid (0, 0, 0);
    for (Vector3D<>& v : data) {
        centroid += v;
    }
    _center = centroid / ((double) data.size ());

    double sum = 0;
    for (Vector3D<>& v : data) {
        double d = MetricUtil::dist2 (_center, v);
        sum += Math::sqr (1 / (d + 1));
    }

    return sum / data.size ();
}
