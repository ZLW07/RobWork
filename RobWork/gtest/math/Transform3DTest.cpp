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
#include <rw/math/Transform3D.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Constants.hpp>


using namespace rw::math;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return v.normInf();
    }
}

TEST(Transform3DTest, Transform3D){
    const Vector3D<> d(1, 2, 3);
    const Transform3D<> t(d);

    const Vector3D<> v1(0, 0, 0);
    EXPECT_EQ(norm_inf(t * v1 - d) , 0);

    const EAA<> eaa(Pi / 2, 0, 0);
    const Rotation3D<> r1 = eaa.toRotation3D();
    const Transform3D<> t2(d, r1);
    const Transform3D<> t3 = t2 * inverse(t2);

    const EAA<> xeaa(t3.R());
    EXPECT_EQ(norm_inf(xeaa.axis()) , 0);
    EXPECT_EQ(xeaa.angle() , 0);
    EXPECT_LT(norm_inf(t3.P()) , 1e-15);

    Transform3D<float> tf;
    tf = cast<float>(t);
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 4; j++)
            EXPECT_EQ(tf(i, j) , (float)t(i, j));
    tf = rw::math::cast<float>(t); // qualified lookup
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 4; j++)
            EXPECT_EQ(tf(i, j) , (float)t(i, j));

    /* Test comparison operators operator== and operator!= */
    const EAA<> eaacomp1(Pi / 2, 0, 0);
    const Rotation3D<> rotcomp1 = eaacomp1.toRotation3D();
    const Vector3D<double> comp1(1.1, -2.2, 3.3);
    const Transform3D<double> tcomp1(comp1, rotcomp1);

    const EAA<> eaacomp2(Pi / 2, 0, 0);
    const Rotation3D<> rotcomp2 = eaacomp2.toRotation3D();
    const Vector3D<double> comp2(1.1, -2.2, 3.3);
    const Transform3D<double> tcomp2(comp2, rotcomp2);

    EXPECT_EQ(tcomp1 , tcomp2);
    EXPECT_TRUE(!(tcomp1 != tcomp2));

    const EAA<> eaacomp3(Pi / 4, 0, 0);
    const Rotation3D<> rotcomp3 = eaacomp3.toRotation3D();
    const Vector3D<double> comp3(1.1, -2.2, 3.3);
    const Transform3D<double> tcomp3(comp3, rotcomp3);
    EXPECT_NE(tcomp1 , tcomp3);
    EXPECT_TRUE(!(tcomp1 == tcomp3));
}
