#################################################################################
 # Copyright 2021 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 # Faculty of Engineering, University of Southern Denmark
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #################################################################################


#from sdurw import *
import unittest                                     # now we can use python unittest framework
import math, sys
import sdurw, sdurw_math


class UtilTest(unittest.TestCase):

    def test_Util(self):
        q = sdurw_math.Quaternion(0.1, 0.2, 0.3, 0.4)
        q.normalize()
        eaa = sdurw_math.EAAd(1,2,3)

        eaa_q = sdurw_math.Math.quaternionToEAA(q)
        q_eaa = sdurw_math.Math.eaaToQuaternion(eaa_q)

        self.assertAlmostEqual(q[0],q_eaa[0], delta = 1e-12)
        self.assertAlmostEqual(q[1],q_eaa[1], delta = 1e-12)
        self.assertAlmostEqual(q[2],q_eaa[2], delta = 1e-12)
        self.assertAlmostEqual(q[3],q_eaa[3], delta = 1e-12)

    def test_testCeilLog2(self):
        self.assertEqual(sdurw_math.Math.ceilLog2(1), 0)
        self.assertEqual(sdurw_math.Math.ceilLog2(2), 1)
        self.assertEqual(sdurw_math.Math.ceilLog2(3), 2)
        self.assertEqual(sdurw_math.Math.ceilLog2(4), 2)
        self.assertEqual(sdurw_math.Math.ceilLog2(5), 3)
        self.assertEqual(sdurw_math.Math.ceilLog2(8), 3)
        self.assertEqual(sdurw_math.Math.ceilLog2(9), 4)

    def test_Vector3D_norm(self):
        v1 = sdurw_math.Vector3Dd(1.0, 2.0, 2.0)
        self.assertEqual(sdurw_math.MetricUtil.norm1(v1), 5.0)
        self.assertEqual(sdurw_math.MetricUtil.norm2(v1), 3.0)
        self.assertEqual(sdurw_math.MetricUtil.normInf(v1), 2.0)

    def test_Vector3D_cross(self):
        v1 = sdurw_math.Vector3Dd(1.0, 2.0, 2.0)
        v2 = sdurw_math.Vector3Dd(v1)                      # Make a copy of v1 using c++ copy constructor

        self.assertEqual(sdurw_math.MetricUtil.normInf(sdurw_math.cross(v1, v2)), 0)

    def test_Transform3DAngleMetric(self):
        v1 = sdurw_math.Vector3Dd(1.0, 2.0, 2.0)
        t1 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(1, 0, 0), sdurw_math.Rotation3Dd.identity())
        t2 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(1, 0, 0), sdurw_math.RPYd(1.4, 0, 0).toRotation3D())

        x1 = sdurw_math.Transform3DAngleMetric(1.0, 0.0).distance(t1, t2)
        x2 = sdurw_math.Transform3DAngleMetric(0.0, 1.0).distance(t1, t2)
        self.assertAlmostEqual(x1, 0, delta = 1e-15)
        self.assertAlmostEqual(x2, 1.4, delta = 1e-15)

    def test_Wrench6D(self):
        wrench = sdurw_math.Wrench6Dd(1,2,3,4,5,6)
        self.assertEqual(wrench.force()[0] , 1)
        self.assertEqual(wrench.force()[1] , 2)
        self.assertEqual(wrench.force()[2] , 3)
        self.assertEqual(wrench.torque()[0] , 4)
        self.assertEqual(wrench.torque()[1] , 5)
        self.assertEqual(wrench.torque()[2] , 6)

    def test_RandomSeedTest(self):

        # The boost random generator uses Mersenne twister pseudo-random generator
	    # The random values should therefore be deterministic for a given seed
        # Check that the correct values are obtained, and that seeding with the same
        # value gives same result.
        for i in range (0,2):
            sdurw_math.Math.seed(10)
            ran = sdurw_math.Math.ran()
            ranFromTo = sdurw_math.Math.ran(-0.2,0.4)
            ranI = sdurw_math.Math.ranI(-200,7000)
            ranNormalDist = sdurw_math.Math.ranNormalDist(1., 2.2)
            ranQ = sdurw_math.Math.ranQ( sdurw_math.Q(2,0.1,0.5) , sdurw_math.Q(2,0.5,1.0))
            ranQpair = sdurw_math.Math.ranQ(sdurw_math.Q(2,0.1,0.5) , sdurw_math.Q(2,0.5,1.0))
            ranDir = sdurw_math.Math.ranDir(4, 2.1)
            ranWeightedDir = sdurw_math.Math.ranWeightedDir(3, sdurw_math.Q(3,1.0,2.0,3.0), 0.1)

            #Comparing with a tolerance of 1.0e-06%. This value was found by looking at the highest difference in % (i.e. 2.01204e-07% in my/mband's case) that was reported when running these tests with a too low tolerance (e.g. 1.0e-09).
            self.assertAlmostEqual(ran, 0.77132064313627779, delta = 1e-06)
            self.assertAlmostEqual(ranFromTo, -0.020743304910138233, delta = 1.0e-06)
            self.assertEqual(ranI,-51)
            self.assertAlmostEqual(ranQ[0], 0.27720597842708228, delta = 1.0e-06)
            self.assertAlmostEqual(ranQ[1], 0.87440194131340832, delta = 1.0e-06)
            self.assertAlmostEqual(ranQpair[0], 0.43276454471051695, delta = 1.0e-06)
            self.assertAlmostEqual(ranQpair[1], 0.74925350688863546, delta = 1.0e-06)

            # Asuming BOOST_VERSION >= 105600    # Uses Ziggurat algorithm
            self.assertAlmostEqual(ranNormalDist, 1.4202744938961578, delta = 1e-06)
            self.assertAlmostEqual(ranDir[0], 1.1510367394955425, delta = 1e-06)
            self.assertAlmostEqual(ranDir[1], 1.320001765811462, delta = 1e-06)
            self.assertAlmostEqual(ranDir[2], -0.060350434510439331, delta = 1e-06)
            self.assertAlmostEqual(ranDir[3], 1.1571808793963436, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[0], -0.01586616670934473, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[1], -0.0079894098272859024, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[2], 0.032477243445557302, delta = 1e-06)

        # Check another seed        
        for i in range (0,2):
            sdurw_math.Math.seed(123456)
            ran = sdurw_math.Math.ran()
            ranFromTo = sdurw_math.Math.ran(0,1)
            ranI = sdurw_math.Math.ranI(-200,7000)
            ranNormalDist = sdurw_math.Math.ranNormalDist(1., 2.2)
            ranQ = sdurw_math.Math.ranQ( sdurw_math.Q(2,0.1,0.5) , sdurw_math.Q(2,0.5,1.0))
            ranQpair = sdurw_math.Math.ranQ(sdurw_math.Q(2,0.1,0.5) , sdurw_math.Q(2,0.5,1.0))
            ranDir = sdurw_math.Math.ranDir(4, 2.1)
            ranWeightedDir = sdurw_math.Math.ranWeightedDir(3, sdurw_math.Q(3,1.0,2.0,3.0), 0.1)

            self.assertAlmostEqual(ran, 0.12696982943452895, delta = 1e-06)
            self.assertAlmostEqual(ranFromTo, 0.5149132558144629, delta = 1.0e-06)
            self.assertEqual(ranI,6760)
            self.assertAlmostEqual(ranQ[0], 0.38232804937288167, delta = 1.0e-06)
            self.assertAlmostEqual(ranQ[1], 0.94861826102714986, delta = 1.0e-06)
            self.assertAlmostEqual(ranQpair[0], 0.41153188152238729, delta = 1.0e-06)
            self.assertAlmostEqual(ranQpair[1], 0.68837485776748508, delta = 1.0e-06)            

            # Asuming BOOST_VERSION >= 105600    # Uses Ziggurat algorithm
            self.assertAlmostEqual(ranNormalDist, 1.403451703065415, delta = 1e-06)
            self.assertAlmostEqual(ranDir[0], -1.0179557466375477, delta = 1e-06)
            self.assertAlmostEqual(ranDir[1], -0.48580514389579094, delta = 1e-06)
            self.assertAlmostEqual(ranDir[2], -0.45149359697442121, delta = 1e-06)
            self.assertAlmostEqual(ranDir[3], -1.7128668926519315, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[0], 0.0132146851122913, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[1], -0.0028023557193167545, delta = 1e-06)
            self.assertAlmostEqual(ranWeightedDir[2], -0.032988144852141181, delta = 1e-06)

    def test_ranRotation3D(self):
        times = 10
        sdurw_math.Math.seed(423)                             # seed the RNG - The seed was arbitrarily chosen
        for i in range (0,times):
            rot = sdurw_math.Math.ranRotation3D()
            self.assertTrue(rot.isProperRotation(1.0e-15))

        for i in range (0,times):
            rot = sdurw_math.Rotation3Df()
            self.assertTrue(rot.isProperRotation(1.0e-06))

    def test_ranTransform3D(self):
        #
        # Not testing the translation part as that should just be some random doubles or floats
        #
        times = 10
        sdurw_math.Math.seed(829)                             # seed the RNG - The seed was arbitrarily chosen
        for i in range (0,times):
            transform = sdurw_math.Math.ranTransform3D()
#            self.assertTrue(transform.R().isProperRotation(1.0e-15)) ERROR on c++ side maby talk with Thomas

        for i in range (0,times):
            transform = sdurw_math.Rotation3Df()
            self.assertTrue(transform.isProperRotation(1.0e-06))


    def test_MatrixVectorConvertionTest(self):
        # Test Math convenience functions for serialization and deserialization of matrix types.
        sdurw_math.Math.seed(829)
        T = sdurw_math.Math.ranTransform3D()
        Tserialized = sdurw_math.Math.toStdVector(T, 3, 4)
        self.assertEqual(len(Tserialized) , 12)
        Trestored = sdurw_math.Transform3Dd()
        sdurw_math.Math.fromStdVectorToMat(Tserialized, Trestored, 3, 4)
        self.assertTrue(T.equal(Trestored))


if __name__ == '__main__':
    unittest.main()    
        