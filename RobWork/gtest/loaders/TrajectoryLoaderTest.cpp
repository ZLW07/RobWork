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


#include <gtest/gtest.h>
#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/trajectory/Path.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>

#include <cmath>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rw::trajectory;
using namespace rw::loaders;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return v.normInf();
    }

    double norm_inf(const Q& v)
    {
        return norm_inf(v.e());
    }

    bool isZero(double x) { return fabs(x) < 1e-14; }
}

TEST(TrajectoryLoaderTest, PathLoaderTest )
{
    QPath path;
    Q q(3);
    for (int i = 0; i < 100; i++) {
        for (size_t j = 0; j < q.size(); j++)
            q(j) = rand();
        path.push_back(q);
    }
    PathLoader::storePath(path, "path.pth");

    QPath path2 = PathLoader::loadPath("path.pth");
    EXPECT_EQ(path.size() , path2.size());

    QPath::iterator it1 = path.begin();
    QPath::iterator it2 = path2.begin();
    for (; it1 != path.end(); ++it1, ++it2) {
        EXPECT_EQ(*it1 ,*it2);
    }
}
