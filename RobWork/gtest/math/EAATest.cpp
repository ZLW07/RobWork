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

#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

#include <gtest/gtest.h>

using namespace rw::math;

TEST (EAA, Test_ADL)
{
    Vector3D<> vec1 (0.1, 0.2, 0.3);
    EAA<> eaa2 (0.4, 0.5, 0.6);
    Vector3D<> c;

    c = cross (vec1, eaa2);
    EXPECT_LT (fabs (c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15);
    EXPECT_LT (fabs (c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15);
    EXPECT_LT (fabs (c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15);
}

TEST (EAA, CastTest)
{
    EAA<> eaacast (0.1, 0.2, 0.3);
    EAA< float > eaaf;
    eaaf = cast< float > (eaacast);
    for (size_t i = 0; i < 3; i++) {
        EXPECT_EQ (eaaf (i), (float) eaacast (i));
    }
    eaaf = rw::math::cast< float > (eaacast);    // qualified lookup
    for (size_t i = 0; i < 3; i++) {
        EXPECT_EQ (eaaf (i), (float) eaacast (i));
    }
}

TEST (EAA, CrossProductTest)
{
    Vector3D<> vec1 (0.1, 0.2, 0.3);
    EAA<> eaa2 (0.4, 0.5, 0.6);
    Vector3D<> c;

    c = cross (vec1, eaa2);
    EXPECT_LT (fabs (c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15);
    EXPECT_LT (fabs (c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15);
    EXPECT_LT (fabs (c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15);
    c = rw::math::cross (vec1, eaa2);    // qualified lookup
    EXPECT_LT (fabs (c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15);
    EXPECT_LT (fabs (c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15);
    EXPECT_LT (fabs (c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15);
}

TEST (EAA, MiscTest)
{
    // 0 degree
    EAA<> e0 (0.0, 0.0, 0.0);
    EXPECT_EQ (e0.angle () , 0);
    EXPECT_EQ ( (e0.axis ()).normInf() , 0);
    EAA<> xe0 (e0.toRotation3D ());
    EXPECT_EQ (xe0.angle () , e0.angle ());
    EXPECT_EQ ((xe0.axis () - e0.axis ()).normInf() , 0);

    // 180 degree
    EAA<> e180_1 (Pi, 0.0, 0.0);
    EXPECT_LT (fabs (e180_1.angle () - Pi) , 1e-16);
    EXPECT_EQ ((e180_1.axis () - Vector3D<> (1.0, 0.0, 0.0)).normInf() , 0);
    EAA<> xe180_1 (e180_1.toRotation3D ());
    EXPECT_LT (fabs (xe180_1.angle () - e180_1.angle ()) , 1e-16);
    EXPECT_EQ ((xe180_1.axis () - e180_1.axis ()).normInf() , 0);

    EAA<> e180_2 (0.0, Pi, 0.0);
    EXPECT_DOUBLE_EQ (e180_2.angle (), Pi);
    EXPECT_EQ ((e180_2.axis () - Vector3D<> (0.0, 1.0, 0.0)).normInf (), 0);
    EAA<> xe180_2 (e180_2.toRotation3D ());
    EXPECT_DOUBLE_EQ (xe180_2.angle (), e180_2.angle ());
    EXPECT_EQ ((xe180_2.axis () - e180_2.axis ()).normInf(), 0);

    EAA<> e180_3 (0.0, 0.0, Pi);
    EXPECT_DOUBLE_EQ (e180_3.angle (), Pi);
    EXPECT_EQ ((e180_3.axis () - Vector3D<> (0.0, 0.0, 1.0)).normInf(), 0);
    EAA<> xe180_3 (e180_3.toRotation3D ());
    EXPECT_DOUBLE_EQ (xe180_3.angle (), e180_3.angle ());
    EXPECT_EQ ((xe180_3.axis () - e180_3.axis ()).normInf(), 0);

    const double val1 = 0.3;
    const double val2 = 0.4;
    const double val3 = 0.5;
    for (int sign1 = -1; sign1 <= 1; sign1++) {
        for (int sign2 = -1; sign2 <= 1; sign2++) {
            for (int sign3 = -1; sign3 <= 1; sign3++) {
                if (sign1 == 0 && sign2 == 0 && sign3 == 0)
                    continue;    // the zero case is not tested here
                Vector3D<> axisInput (sign1 * val1, sign2 * val2, sign3 * val3);
                axisInput = normalize (axisInput);
                EAA<> e180 (axisInput * Pi);
                EXPECT_DOUBLE_EQ (e180.angle (), Pi);
                Vector3D<> e180axis = e180.axis ();
                EXPECT_NEAR ((e180axis - axisInput).normInf (), 0.0, 1e-15);
                EAA<> xe180 (e180.toRotation3D ());
                EXPECT_NEAR (xe180.angle (), e180.angle (), 1e-13);
                Vector3D<> xe180axis = xe180.axis ();
                // vector can point both ways and still be valid
                if ((xe180axis - e180axis).norm2 () > (-xe180axis - e180axis).norm2 ())
                    xe180axis = -xe180axis;
                EXPECT_NEAR ((xe180axis - e180axis).normInf (), 0.0, 1e-15);
            }
        }
    }
    //-- Testing different sign combinations for 180 degree rotations - epsilon
    const double eps = 1e-7;    // should be less than the hardcoded threshold in EAA.cpp
    for (int sign1 = -1; sign1 <= 1; sign1++) {
        for (int sign2 = -1; sign2 <= 1; sign2++) {
            for (int sign3 = -1; sign3 <= 1; sign3++) {
                if (sign1 == 0 && sign2 == 0 && sign3 == 0)
                    continue;    // the zero case is not tested here
                Vector3D<> axisInput (sign1 * val1, sign2 * val2, sign3 * val3);
                axisInput = normalize (axisInput);
                EAA<> e180 (axisInput * (Pi - eps));
                EXPECT_NEAR (e180.angle (), Pi - eps, 1e-13);
                Vector3D<> e180axis = e180.axis ();
                EXPECT_NEAR ((e180axis - axisInput).normInf (), 0.0, 1e-15);
                EAA<> xe180 (e180.toRotation3D ());
                EXPECT_NEAR (xe180.angle (), e180.angle (), 5e-13);
                Vector3D<> xe180axis = xe180.axis ();
                EXPECT_NEAR ((xe180axis - e180axis).normInf (), 0.0, 1e-7);
            }
        }
    }

    // Testing different sign combinations for 180 degree rotations + epsilon
    for (int sign1 = -1; sign1 <= 1; sign1++) {
        for (int sign2 = -1; sign2 <= 1; sign2++) {
            for (int sign3 = -1; sign3 <= 1; sign3++) {
                if (sign1 == 0 && sign2 == 0 && sign3 == 0)
                    continue;    // the zero case is not tested here
                Vector3D<> axisInput (sign1 * val1, sign2 * val2, sign3 * val3);
                axisInput = normalize (axisInput);
                EAA<> e180 (axisInput * (Pi + eps));
                EXPECT_NEAR (e180.angle (), Pi + eps, 1e-13);
                Vector3D<> e180axis = e180.axis ();
                EXPECT_NEAR ((e180axis - axisInput).normInf(),0.0, 1e-15);
                EAA<> xe180 (e180.toRotation3D ());
                EXPECT_NEAR (xe180.angle (), Pi - eps, 5e-13);    // should choose angle < Pi
                Vector3D<> xe180axis = xe180.axis ();
                EXPECT_NEAR ((xe180axis + e180axis).normInf(), 0.0,
                                   1e-7);    // should flip vector to get angle < Pi
            }
        }
    }

    // 90 degree's around x axis
    Vector3D<> v1 (1.0, 0.0, 0.0);
    EAA<> e1 (v1, Pi / 2.0);
    EXPECT_DOUBLE_EQ (e1.angle (), Pi / 2.0);
    EXPECT_EQ ((e1.axis () - v1).normInf (), 0);
    Rotation3D<> rot = e1.toRotation3D ();
    EAA<> e2 (rot);
    EXPECT_EQ ((e1.axis () - e2.axis ()).normInf (), 0);
    EXPECT_DOUBLE_EQ (e1.angle (), e2.angle ());

    /* Test comparison operators operator== and operator!= */
    const EAA< double > comp1 (1.1, -2.2, 3.3);
    const EAA< double > comp2 (1.1, -2.2, 3.3);
    EXPECT_TRUE (comp1 == comp2);
    EXPECT_FALSE (comp1 != comp2);
    const EAA< double > comp3 (1.1, 2.2, -3.3);
    EXPECT_TRUE (comp1 != comp3);
    EXPECT_FALSE (comp1 == comp3);

    /** Test different "problematic setups which previously has resulted in NaN errors */

    Transform3D<> t0 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (0.9990961814,
                                    -0.0003891806,
                                    -0.0425144844,
                                    0.0004163897,
                                    0.9999992493,
                                    0.0006269312,
                                    0.0425143000,
                                    -0.0006440670,
                                    0.9990960000));
    Transform3D<> t1 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (-0.0000000000,
                                    -0.0000000000,
                                    1.0000000000,
                                    0.0000000000,
                                    -1.0000000000,
                                    -0.0000000000,
                                    1.0000000000,
                                    0.0000000000,
                                    0.0000000000));
    Rotation3D<> r0 ((inverse (t0) * t1).R ());
    r0.normalize ();
    EAA<> eaa0 (r0);
    EXPECT_FALSE (Math::isNaN (eaa0.angle ()));

    Transform3D<> t2 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (-1.0000000000,
                                    -0.0000000000,
                                    0.0000000000,
                                    0.0000000000,
                                    -1.0000000000,
                                    0.0000000000,
                                    -0.0000000000,
                                    0.0000000000,
                                    1.0000000000));
    Rotation3D<> r1 ((inverse (t0) * t2).R ());
    r1.normalize ();
    EAA<> eaa1 (r1);
    EXPECT_FALSE (Math::isNaN (eaa1.angle ()));

    Transform3D<> t3 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (1.0000000000,
                                    -0.0000000000,
                                    -0.0000000000,
                                    -0.0000000000,
                                    -1.0000000000,
                                    0.0000000000,
                                    -0.0000000000,
                                    -0.0000000000,
                                    -1.0000000000));
    Rotation3D<> r2 ((inverse (t0) * t3).R ());
    r2.normalize ();
    EAA<> eaa2 (r2);
    EXPECT_FALSE (Math::isNaN (eaa2.angle ()));

    Transform3D<> t4 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (-0.9993793864,
                                    -0.0352195084,
                                    -0.0004122380,
                                    -0.0004294999,
                                    0.0004964944,
                                    0.9999993429,
                                    -0.0352193000,
                                    0.9993790000,
                                    -0.0005113220));
    Transform3D<> t5 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (0.0000000000,
                                    -1.0000000000,
                                    -0.0000000000,
                                    0.0000000000,
                                    0.0000000000,
                                    -1.0000000000,
                                    1.0000000000,
                                    0.0000000000,
                                    0.0000000000));
    Rotation3D<> r3 ((inverse (t4) * t5).R ());
    r3.normalize ();
    EAA<> eaa3 (r3);
    EXPECT_FALSE (Math::isNaN (eaa3.angle ()));

    Transform3D<> t6 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (0.0000000000,
                                    1.0000000000,
                                    0.0000000000,
                                    0.0000000000,
                                    0.0000000000,
                                    -1.0000000000,
                                    -1.0000000000,
                                    0.0000000000,
                                    0.0000000000));
    Rotation3D<> r4 ((inverse (t4) * t6).R ());
    r4.normalize ();
    EAA<> eaa4 (r4);
    EXPECT_FALSE (Math::isNaN (eaa4.angle ()));

    Transform3D<> t7 (Vector3D<> (0.0004406670, 0.1325120000, 0.0236778000),
                      Rotation3D<> (-1.0000000000,
                                    0.0000000000,
                                    0.0000000000,
                                    -0.0000000000,
                                    0.0000000000,
                                    -1.0000000000,
                                    -0.0000000000,
                                    -1.0000000000,
                                    -0.0000000000));
    Rotation3D<> r5 ((inverse (t4) * t7).R ());
    r5.normalize ();
    EAA<> eaa5 (r5);
    EXPECT_FALSE (Math::isNaN (eaa5.angle ()));
}

TEST (EAA, scalarOperatorTest)
{
    EAA<> obj (3, 3, 3);

    auto test1 = obj.elemMultiply(2);
    auto test2 = obj.elemDivide(2);
    auto test3 = obj.elemAdd(2);
    auto test4 = obj.elemSubtract(2);


    for (size_t i = 0; i < obj.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 6.0);
        EXPECT_DOUBLE_EQ (test2[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test3[i], 5.0);
        EXPECT_DOUBLE_EQ (test4[i], 1.0);
    }
}

TEST (EAA, Vector3DOperatorTest)
{
    EAA<> obj1 (3, 3, 3);
    Vector3D<> obj2 (2, 2, 2);

    auto test1 = obj1.elemMultiply(obj2);
    auto test2 = obj1.elemDivide(obj2);
    auto test3 = obj1 + obj2;
    auto test3x = obj2 + obj1;
    auto test4 = obj1 - obj2;
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
    obj2 = obj1.toVector3D();
    EXPECT_TRUE (obj1 == obj2);
    EXPECT_TRUE (obj2 == obj1);
    EXPECT_FALSE (obj1 != obj2);
    EXPECT_FALSE (obj2 != obj1);
}

TEST (EAA, EAAOperatorTest)
{
    EAA<> obj1 (3, 3, 3);
    EAA<> obj2 (2, 2, 2);

    auto test1 = obj1 * obj2;
    auto test2 = obj1.elemMultiply(obj2);
    auto test3 = obj1.elemDivide(obj2);
    auto test4 = obj1.elemAdd(obj2);
    auto test5 = obj1.elemSubtract(obj2);
    auto test7 = obj1;
    test7 += obj2;
    auto test8 = obj1;
    test8 -= obj2;

    for (size_t i = 0; i < obj1.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 1.3724012715315643);
        EXPECT_DOUBLE_EQ (test2[i], 6.0);
        EXPECT_DOUBLE_EQ (test3[i], 3.0/2.0);
        EXPECT_DOUBLE_EQ (test4[i], 5.0);
        EXPECT_DOUBLE_EQ (test5[i], 1.0);
        EXPECT_DOUBLE_EQ (test7[i], 5.0);
        EXPECT_DOUBLE_EQ (test8[i], 1.0);
    }

    const EAA< double > comp1 (1.1, -2.2, 3.3);
    auto comp2 = comp1;
    auto comp3 = -comp1;
    EXPECT_TRUE (comp1 == comp2);
    EXPECT_FALSE (comp1 != comp2);
    EXPECT_TRUE (comp1 != comp3);
    EXPECT_FALSE (comp1 == comp3);
}

TEST (EAA, EigenOperatorTest)
{
    EAA<> obj1 (3, 3, 3);
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

TEST (EAA, MathOperators)
{
    EAA<> obj1 (1, 2, 3);
    EAA<> obj2 (3, 2, 1);
    Vector3D<> obj3 (1,2,3);
    Vector3D<> obj4 (3,2,1);


    auto test1 = obj1.cross (obj2);
    auto test2 = obj1.dot (obj2);
    auto test3 = obj1.cross (obj4);
    auto test4 = obj1.dot (obj4);
    auto test5 = obj3.cross (obj2.toVector3D());
    auto test6 = obj3.dot (obj2.toVector3D());

    EXPECT_EQ (test1,obj1.e ().cross (obj2.e ()));
    EXPECT_EQ (test2,obj1.e ().dot (obj2.e ()));
    EXPECT_EQ (test3, test1);
    EXPECT_EQ (test4, test2);
    EXPECT_EQ (test5, test1);
    EXPECT_EQ (test6, test2);
}
