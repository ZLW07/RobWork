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

#include <rw/math/VectorND.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

using namespace rw::math;

template< size_t N > VectorND< N > getVectorND (double value)
{
    VectorND< N > vec;
    for (size_t i = 0; i < vec.size (); i++) {
        vec[i] = value;
    }
    return vec;
}

template< size_t N > VectorND< N > getVectorND (bool countUp, bool Digit=true)
{
    VectorND< N > vec;
    for (size_t i = 0; i < vec.size (); i++) {
        if(countUp){
            vec[i] = i+1+ Digit*(i+1)%10/10.0;
        }else {
            vec[i] = i - vec.size() +Digit*(i+vec.size())%10/10.0 ;
        }
    }
    return vec;
}

TEST(VectorND,Constructor){
    VectorND< 3 > obj1(1,2,3);
    EXPECT_EQ(obj1[0],1);
    EXPECT_EQ(obj1[1],2);
    EXPECT_EQ(obj1[2],3);

    VectorND< 9 > obj2(1,2.1212142,long(3),4.0,float(5),6,double(7),8u,9);
    EXPECT_DOUBLE_EQ(obj2[0],1);
    EXPECT_DOUBLE_EQ(obj2[1],2.1212142);
    EXPECT_DOUBLE_EQ(obj2[2],3);
    EXPECT_DOUBLE_EQ(obj2[3],4);
    EXPECT_DOUBLE_EQ(obj2[4],5);
    EXPECT_DOUBLE_EQ(obj2[5],6);
    EXPECT_DOUBLE_EQ(obj2[6],7);
    EXPECT_DOUBLE_EQ(obj2[7],8);
    EXPECT_DOUBLE_EQ(obj2[8],9);
}

template< typename T > class VectorNDTest : public ::testing::Test
{};

using test_types = ::testing::Types<
    std::integral_constant< std::size_t, 2 >, std::integral_constant< std::size_t, 3 >,
    std::integral_constant< std::size_t, 4 >, std::integral_constant< std::size_t, 5 >,
    std::integral_constant< std::size_t, 6 >, std::integral_constant< std::size_t, 13 > >;

TYPED_TEST_CASE (VectorNDTest, test_types);

TYPED_TEST (VectorNDTest, MiscTest)
{
    static constexpr std::size_t N = TypeParam::value;
    const VectorND< N > v1 =getVectorND<N>(true,false);
    const VectorND< N > v2 (v1);
    const VectorND< N > v3 = v1 + v2;
    const VectorND< N > v4 = getVectorND<N>(true,false)*2;
    EXPECT_EQ ((v3 - v4).normInf (), 0);

    const VectorND< N > v5 = getVectorND<N>(true);
    const VectorND< N > v5_norm = normalize (v5);
    EXPECT_LT (fabs (v5_norm.norm2 () - 1), 1e-15);

    const double len = v5.norm2 ();
    EXPECT_LT (fabs (v5_norm (0) - v5 (0) / len), 1e-15);
    EXPECT_LT (fabs (v5_norm (1) - v5 (1) / len), 1e-15);
    (fabs (v5_norm (2) - v5 (2) / len) < 1e-15);

    VectorND< N > v6;
    v6 (0) = len;
    EXPECT_EQ (v6 (0), len);

    const VectorND< N, double > vd = getVectorND<N>(true);
    const VectorND< N, int > vi = cast< int > (vd);
    for(int i = 1; size_t (i) <= vi.size(); i++){
        EXPECT_EQ(vi(i-1), i);
    }
}

TYPED_TEST (VectorNDTest, scalarOperatorTest)
{
    static constexpr std::size_t N = TypeParam::value;
    VectorND< N > obj = getVectorND<N>(3.0);

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
    auto test13 = -obj;

    for (size_t i = 0; i < obj.size (); i++) {
        EXPECT_DOUBLE_EQ (test1[i], 6.0);
        EXPECT_DOUBLE_EQ (test2[i], 6.0);
        EXPECT_DOUBLE_EQ (test3[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test4[i], 2.0 / 3.0);
        EXPECT_DOUBLE_EQ (test5[i], 5.0);
        EXPECT_DOUBLE_EQ (test7[i], 1.0);
        EXPECT_DOUBLE_EQ (test9[i], 6.0);
        EXPECT_DOUBLE_EQ (test10[i], 3.0 / 2.0);
        EXPECT_DOUBLE_EQ (test13[i], -3.0);
    }
}

TYPED_TEST (VectorNDTest, VectorNDOperatorTest)
{
    static constexpr std::size_t N = TypeParam::value;
    VectorND< N > obj1 = getVectorND<N>(3.0);
    VectorND< N > obj2 = getVectorND<N>(2.0);

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

TYPED_TEST (VectorNDTest, EigenOperatorTest)
{
    static constexpr std::size_t N = TypeParam::value;
    VectorND< N > obj1             = getVectorND< N > (3.0);
    Eigen::Matrix< double, N, 1 > obj2;
    for (size_t i = 0; i < N; i++) {
        obj2[i] = 2.0;
    }

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

TYPED_TEST (VectorNDTest, ComparisonTest)
{
    static constexpr std::size_t N = TypeParam::value;
    const VectorND< N, double > comp1 (getVectorND<N>(true));
    auto comp2 = comp1;
    auto comp3 = -comp1;
    EXPECT_TRUE (comp1 == comp2);
    EXPECT_FALSE (comp1 != comp2);
    EXPECT_TRUE (comp1 != comp3);
    EXPECT_FALSE (comp1 == comp3);
}

TYPED_TEST (VectorNDTest, MathOperators)
{
    static constexpr std::size_t N = TypeParam::value;
    VectorND< N > obj1 = getVectorND<N>(true);
    VectorND< N > obj2 = getVectorND<N>(false);

    auto test2 = obj1.dot (obj2);
    auto test3 = obj1.normalize ();

    EXPECT_EQ (obj1.e ().dot (obj2.e ()), test2);
    EXPECT_DOUBLE_EQ (test3.norm2 (), 1.0);
    for(size_t i = 0; i < N; i++){
        EXPECT_DOUBLE_EQ((test3 * obj1.norm2 ())[i],obj1[i]);
    }
}
