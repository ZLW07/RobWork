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

#include <rw/math/Vector3D.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

using namespace rw::math;

TEST (Vector3D, MiscTest)
{
    const Vector3D<> v1 (1.0, 2.0, 3.0);
    const Vector3D<> v2 (v1);
    const Vector3D<> v3 = v1 + v2;
    const Vector3D<> v4 = Vector3D<> (2.0, 4.0, 6.0);
    EXPECT_EQ ((v3 - v4).normInf (), 0);

    const Vector3D<> v5 (3.0, 4.0, 5.0);
    const Vector3D<> v5_norm = normalize (v5);
    EXPECT_LT (fabs (v5_norm.norm2 () - 1), 1e-15);

    const double len = v5.norm2 ();
    EXPECT_LT (fabs (v5_norm (0) - v5 (0) / len), 1e-15);
    EXPECT_LT (fabs (v5_norm (1) - v5 (1) / len), 1e-15);
    (fabs (v5_norm (2) - v5 (2) / len) < 1e-15);

    Vector3D<> v6;
    v6 (0) = len;
    EXPECT_EQ (v6 (0), len);

    const Vector3D< std::string > vs1 ("x1", "y1", "z1");

    EXPECT_EQ (vs1[0], "x1");
    EXPECT_EQ (vs1[1], "y1");
    EXPECT_EQ (vs1[2], "z1");

    EXPECT_EQ (cross (v1, v2).normInf (), 0);

    const Vector3D< double > vd (1.1, 5.51, -10.3);
    const Vector3D< int > vi = cast< int > (vd);
    EXPECT_EQ (vi (0), 1);
    EXPECT_EQ (vi (1), 5);
    EXPECT_EQ (vi (2), -10);

    /* Test comparison operators operator== and operator!= */
    const Vector3D< double > comp1 (1.1, -2.2, 3.3);
    const Vector3D< double > comp2 (1.1, -2.2, 3.3);
    EXPECT_TRUE (comp1 == comp2);
    EXPECT_TRUE (!(comp1 != comp2));
    const Vector3D< double > comp3 (1.1, 2.2, -3.3);
    EXPECT_TRUE (comp1 != comp3);
    EXPECT_TRUE (!(comp1 == comp3));
}

TEST (Vector3D, scalarOperatorTest)
{
    Vector3D<> obj (3, 3, 3);

    auto test1 = obj * 2;
    auto test2 = 2 * obj;
    auto test3 = obj / 2;
    auto test4 = 2 / obj;
    auto test5 = obj.elemAdd(2);
    auto test7 = obj.elemSubtract(2);
    auto test9 = obj;
    test9 *= 2;
    auto test10 = obj;
    test10 /= 2;
    for (size_t i = 0; i < obj.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 6.0);
        EXPECT_DOUBLE_EQ (test2[i], 6.0);
        EXPECT_DOUBLE_EQ (test3[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test4[i], 2.0 / 3.0);
        EXPECT_DOUBLE_EQ (test5[i], 5.0);
        EXPECT_DOUBLE_EQ (test7[i], 1.0);
        EXPECT_DOUBLE_EQ (test9[i], 6.0);
        EXPECT_DOUBLE_EQ (test10[i], 3.0 / 2.0);
    }
}

TEST (Vector3D, Vector3DOperatorTest)
{
    Vector3D<> obj1 (3, 3, 3);
    Vector3D<> obj2 (2, 2, 2);

    auto test1 = obj1.elemMultiply(obj2);
    auto test2 = obj1.elemDivide(obj2);
    auto test3 = obj1 + obj2;
    auto test4 = obj1 - obj2;
    auto test7 = obj1;
    test7 += obj2;
    auto test8 = obj1;
    test8 -= obj2;

    for (size_t i = 0; i < obj1.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 6.0);
        EXPECT_DOUBLE_EQ (test2[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test3[i], 5.0);
        EXPECT_DOUBLE_EQ (test4[i], 1.0);
        EXPECT_DOUBLE_EQ (test7[i], 5.0);
        EXPECT_DOUBLE_EQ (test8[i], 1.0);
    }
}

TEST (Vector3D, EigenOperatorTest)
{
    Vector3D<> obj1 (3, 3, 3);
    Eigen::Vector3d obj2 (2, 2, 2);

    auto test1  = obj1.elemMultiply(obj2);
    auto test2  = obj1.elemDivide(obj2);
    auto test3  = obj1 + obj2;
    auto test3x = obj2 + obj1;
    auto test4  = obj1 - obj2;
    auto test4x = obj2 - obj1;
    auto test7 = obj1;
    test7 += obj2;
    auto test8 = obj1;
    test8 -= obj2;

    for (size_t i = 0; i < obj1.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 6.0);
        EXPECT_DOUBLE_EQ (test2[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test3[i], 5.0);
        EXPECT_DOUBLE_EQ (test3x[i], 5.0);
        EXPECT_DOUBLE_EQ (test4[i], 1.0);
        EXPECT_DOUBLE_EQ (test4x[i], -1.0);
        EXPECT_DOUBLE_EQ (test7[i], 5.0);
        EXPECT_DOUBLE_EQ (test8[i], 1.0);
    }

    EXPECT_TRUE (obj1 != obj2);
    EXPECT_TRUE (obj2 != obj1);
    EXPECT_FALSE (obj1 == obj2);
    EXPECT_FALSE (obj2 == obj1);
    obj2 = obj1.e();
    EXPECT_TRUE (obj1 == obj2);
    EXPECT_TRUE (obj2 == obj1);
    EXPECT_FALSE (obj1 != obj2);
    EXPECT_FALSE (obj2 != obj1);
}

TEST (Vector3D, ComparisonTest)
{
    const Vector3D< double > comp1 (1.1, -2.2, 3.3);
    auto comp2 = comp1;
    auto comp3 = -comp1;
    EXPECT_TRUE (comp1 == comp2);
    EXPECT_FALSE (comp1 != comp2);
    EXPECT_TRUE (comp1 != comp3);
    EXPECT_FALSE (comp1 == comp3);
}

TEST (Vector3D, MathOperators)
{
    Vector3D<> obj1 (1, 2, 3);
    Vector3D<> obj2 (3, 2, 1);

    auto test1 = obj1.cross (obj2);
    auto test2 = obj1.dot (obj2);
    auto test3 = obj1.normalize ();

    EXPECT_EQ (obj1.e ().cross (obj2.e ()), test1);
    EXPECT_EQ (obj1.e ().dot (obj2.e ()), test2);
    EXPECT_EQ (test3.norm2 (), 1.0);
    EXPECT_EQ (test3 * obj1.norm2 (), obj1);
}
