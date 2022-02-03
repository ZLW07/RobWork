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
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/Wrench6D.hpp>

#include <boost/version.hpp>

using namespace rw::math;

TEST(UtilTest, Util)
{
    Quaternion<> q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    const EAA<> eaa(1,2,3);

    const EAA<> eaa_q = Math::quaternionToEAA(q);
    const Quaternion<> q_eaa = Math::eaaToQuaternion(eaa_q);

    EXPECT_NEAR(q(0),q_eaa(0), 1e-12);
    EXPECT_NEAR(q(1),q_eaa(1), 1e-12);
    EXPECT_NEAR(q(2),q_eaa(2), 1e-12);
    EXPECT_NEAR(q(3),q_eaa(3), 1e-12);
}

TEST(UtilTest, testCeilLog2)
{
    EXPECT_EQ(Math::ceilLog2(1), 0);
    EXPECT_EQ(Math::ceilLog2(2), 1);
    EXPECT_EQ(Math::ceilLog2(3), 2);
    EXPECT_EQ(Math::ceilLog2(4), 2);
    EXPECT_EQ(Math::ceilLog2(5), 3);
    EXPECT_EQ(Math::ceilLog2(8), 3);
    EXPECT_EQ(Math::ceilLog2(9), 4);
}

TEST(UtilTest, testVector3D_norm){
    Vector3D<> v1(1.0, 2.0, 2.0);

    EXPECT_EQ( MetricUtil::norm1(v1), 5.0);
    EXPECT_EQ( MetricUtil::norm2(v1), 3.0);
    EXPECT_EQ( MetricUtil::normInf(v1), 2.0);
}

TEST(UtilTest, testVector3D_cross){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);

    EXPECT_EQ( MetricUtil::normInf(cross(v1, v2)), 0);
}

TEST(UtilTest, testTransform3DAngleMetric){
    Vector3D<> v1(1.0, 2.0, 2.0);
	Transform3D<> t1(Vector3D<>(1, 0, 0), Rotation3D<>::identity());
    Transform3D<> t2(Vector3D<>(1, 0, 0), RPY<>(1.4, 0, 0).toRotation3D());
	EXPECT_NEAR(Transform3DAngleMetric<double>(1.0, 0.0).distance(t1, t2), 0, 1e-15);
	EXPECT_NEAR(Transform3DAngleMetric<double>(0.0, 1.0).distance(t1, t2), 1.4, 1e-15);
}


TEST(UtilTest, testRotation3D_inverse){
    Vector3D<std::string> i("i1", "i2", "i3");
    Vector3D<std::string> j("j1", "j2", "j3");
    Vector3D<std::string> k("k1", "k2", "k3");
    Rotation3D<std::string> rot(i, j, k);
    EXPECT_EQ(rot(1,0) , "i2");
    EXPECT_EQ(inverse(rot)(1,0) , "j1");

    Rotation3D<std::string> rotInv(inverse(rot));
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            EXPECT_EQ(rot(r,c) , rotInv(c,r));
        }
    }
}

TEST(UtilTest, testWrench6D){
	rw::math::Wrench6D<> wrench(1,2,3,4,5,6);
	EXPECT_EQ(wrench.force()[0] , 1 );
	EXPECT_EQ(wrench.force()[1] , 2 );
	EXPECT_EQ(wrench.force()[2] , 3 );
	EXPECT_EQ(wrench.torque()[0] , 4 );
	EXPECT_EQ(wrench.torque()[1] , 5 );
	EXPECT_EQ(wrench.torque()[2] , 6 );

}

TEST(UtilTest, RandomSeedTest)
{
	// The boost random generator uses Mersenne twister pseudo-random generator
	// The random values should therefore be deterministic for a given seed
    // Check that the correct values are obtained, and that seeding with the same
    // value gives same result.
    for (unsigned int i = 0; i < 2; i++) {
    	Math::seed(10);
    	const double ran = Math::ran();
    	const double ranFromTo = Math::ran(-0.2,0.4);
    	const int ranI = Math::ranI(-200,7000);
    	const double ranNormalDist = Math::ranNormalDist(1., 2.2);
    	const Q ranQ = Math::ranQ(Q(2,0.1,0.5),Q(2,0.5,1.0));
    	const Q ranQpair = Math::ranQ(std::make_pair<Q,Q>(Q(2,0.1,0.5),Q(2,0.5,1.0)));
    	const Q ranDir = Math::ranDir(4, 2.1);
    	const Q ranWeightedDir = Math::ranWeightedDir(3, Q(3,1.0,2.0,3.0), 0.1);
        // Comparing with a tolerance of 1.0e-06%. This value was found by looking at the highest difference in % (i.e. 2.01204e-07% in my/mband's case) that was reported when running these tests with a too low tolerance (e.g. 1.0e-09).
   	EXPECT_NEAR(ran,0.77132064313627779,1.0e-06);
    	EXPECT_NEAR(ranFromTo,-0.020743304910138233,1.0e-06);
    	EXPECT_EQ(ranI,-51);
    	EXPECT_NEAR(ranQ[0],0.27720597842708228,1.0e-06);
    	EXPECT_NEAR(ranQ[1],0.87440194131340832,1.0e-06);
    	EXPECT_NEAR(ranQpair[0],0.43276454471051695,1.0e-06);
    	EXPECT_NEAR(ranQpair[1],0.74925350688863546,1.0e-06);

		// Values depending on normal distribution (can be different according to Boost version)
#if BOOST_VERSION >= 105600 // Uses Ziggurat algorithm
		EXPECT_NEAR(ranNormalDist, 1.4202744938961578, 1.0e-06);
		EXPECT_NEAR(ranDir[0], 1.1510367394955425, 1.0e-06);
		EXPECT_NEAR(ranDir[1], 1.320001765811462, 1.0e-06);
		EXPECT_NEAR(ranDir[2], -0.060350434510439331, 1.0e-06);
		EXPECT_NEAR(ranDir[3], 1.1571808793963436, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[0], -0.01586616670934473, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[1], -0.0079894098272859024, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[2], 0.032477243445557302, 1.0e-06);
#else // Uses Box-Muller algorithm
		EXPECT_NEAR(ranNormalDist, -2.1159354853352048, 1.0e-06);
    	EXPECT_NEAR(ranDir[0],0.10420574144754288,1.0e-06);
    	EXPECT_NEAR(ranDir[1],-1.3371934765112323,1.0e-06);
    	EXPECT_NEAR(ranDir[2],-0.77189916796530089,1.0e-06);
    	EXPECT_NEAR(ranDir[3],1.4195867160267628,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[0],0.0021004772778212945,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[1],-0.0086033368952058847,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[2],-0.03282871096443158,1.0e-06);
#endif
    }
    // Check another seed
    for (unsigned int i = 0; i < 2; i++) {
    	Math::seed(123456);
    	const double ran = Math::ran();
    	const double ranFromTo = Math::ran(0,1);
    	const int ranI = Math::ranI(-200,7000);
    	const double ranNormalDist = Math::ranNormalDist(1., 2.2);
    	const Q ranQ = Math::ranQ(Q(2,0.1,0.5),Q(2,0.5,1.0));
    	const Q ranQpair = Math::ranQ(std::make_pair<Q,Q>(Q(2,0.1,0.5),Q(2,0.5,1.0)));
    	const Q ranDir = Math::ranDir(4, 2.1);
    	const Q ranWeightedDir = Math::ranWeightedDir(3, Q(3,1.0,2.0,3.0), 0.1);
    	EXPECT_NEAR(ran,0.12696982943452895,1.0e-06);
    	EXPECT_NEAR(ranFromTo,0.5149132558144629,1.0e-06);
    	EXPECT_EQ(ranI,6760);
    	EXPECT_NEAR(ranQ[0],0.38232804937288167,1.0e-06);
    	EXPECT_NEAR(ranQ[1],0.94861826102714986,1.0e-06);
    	EXPECT_NEAR(ranQpair[0],0.41153188152238729,1.0e-06);
    	EXPECT_NEAR(ranQpair[1],0.68837485776748508,1.0e-06);

		// Values depending on normal distribution (can be different according to Boost version)
#if BOOST_VERSION >= 105600 // Uses Ziggurat algorithm
		EXPECT_NEAR(ranNormalDist, 1.403451703065415, 1.0e-06);
		EXPECT_NEAR(ranDir[0], -1.0179557466375477, 1.0e-06);
		EXPECT_NEAR(ranDir[1], -0.48580514389579094, 1.0e-06);
		EXPECT_NEAR(ranDir[2], -0.45149359697442121, 1.0e-06);
		EXPECT_NEAR(ranDir[3], -1.7128668926519315, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[0], 0.0132146851122913, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[1], -0.0028023557193167545, 1.0e-06);
		EXPECT_NEAR(ranWeightedDir[2], -0.032988144852141181, 1.0e-06);
#else // Uses Box-Muller algorithm
		EXPECT_NEAR(ranNormalDist, 2.3707402760475729, 1.0e-06);
    	EXPECT_NEAR(ranDir[0],-0.76867727692667076,1.0e-06);
    	EXPECT_NEAR(ranDir[1],1.3635143841520583,1.0e-06);
    	EXPECT_NEAR(ranDir[2],-0.62472325234244208,1.0e-06);
    	EXPECT_NEAR(ranDir[3],-1.2528705544188166,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[0],-0.014881418304403815,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[1],-0.019235042703143274,1.0e-06);
    	EXPECT_NEAR(ranWeightedDir[2],0.030365543188295908,1.0e-06);
#endif
    }
}

TEST(UtilTest, ranRotation3D) {
    const int times = 10;
    Math::seed(423); // seed the RNG - The seed was arbitrarily chosen
    for (int i = 0; i < times; ++i) {
        Rotation3D<> rot = Math::ranRotation3D<double>();
        EXPECT_TRUE(rot.isProperRotation(1.0e-15));
        // Use the following to have the value of the determinant printed, if the precision of the comparison within the implemented functions used by isProperRotation(), has to be updated.
        // BOOST_CHECK_EQUAL(rot.e().determinant(), 1.0);
    }

    for (int i = 0; i < times; ++i) {
        Rotation3D<float> rot = Math::ranRotation3D<float>();
        EXPECT_TRUE(rot.isProperRotation(1.0e-06f));
        // BOOST_CHECK_EQUAL(rot.e().determinant(), 1.0);
    }
}

TEST(UtilTest, ranTransform3D) {
    /*
     * Not testing the translation part as that should just be some random doubles or floats
     */
    const int times = 10;
    Math::seed(829); // seed the RNG - The seed was arbitrarily chosen
    for (int i = 0; i < times; ++i) {
        Transform3D<> transform = Math::ranTransform3D<double>();
        EXPECT_TRUE(transform.R().isProperRotation(1.0e-15));
    }

    for (int i = 0; i < times; ++i) {
        Transform3D<float> transform = Math::ranTransform3D<float>();
        EXPECT_TRUE(transform.R().isProperRotation(1.0e-06f));
    }
}

TEST(UtilTest, MatrixVectorConvertionTest) {
    // Test Math convenience functions for serialization and deserialization of matrix types.
    Math::seed(829);
    const Transform3D<> T = Math::ranTransform3D<double>();
    std::vector<double> Tserialized = Math::toStdVector(T, 3, 4);
    EXPECT_EQ(Tserialized.size() , 12);
    Transform3D<> Trestored;
    Math::fromStdVectorToMat(Tserialized, Trestored, 3, 4);
    EXPECT_TRUE(T.equal(Trestored));
}

TEST(UtilTest, NanFunctionality) {
    double aNumber = 0.3;
    double aNaN = Math::NaN();
    EXPECT_TRUE(!Math::isNaN(aNumber));
    EXPECT_TRUE(Math::isNaN(aNaN));
}
