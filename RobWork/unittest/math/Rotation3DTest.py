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

import unittest                                     # now we can use python unittest framework
import math, sys
import sdurw_math

# Function to test exception when creating Q with 11 arguments.
# What a ugly way to do this...
# Link https://ongspxm.gitlab.io/blog/2016/11/assertraises-testing-for-errors-in-unittest/
#
def qc10_func():
    qc10 = sdurw_math.Q(10,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2)


class Rotation3DTest(unittest.TestCase):

    def test_MetricTest(self):
        v21 = sdurw_math.Vector2Dd(0.1, 0.2)
        v31 = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        q1 = sdurw_math.Q(3, 0.1, 0.2, 0.3)
        q2 = sdurw_math.Q(3, 0.1, 0.2, 0.3)

        mh1 = sdurw_math.ManhattanMetricVector2D()
        mh2 = sdurw_math.ManhattanMetricVector3D()
        mh3 = sdurw_math.ManhattenMatricQ()

        self.assertAlmostEqual(0.1+0.2, mh1.distance(v21), delta = 0.00001)
        self.assertAlmostEqual(0.1+0.2+0.3, mh2.distance(v31), delta = 0.00001)
        self.assertAlmostEqual(0.1+0.2+0.3, mh3.distance(q1), delta = 0.00001)
        self.assertAlmostEqual(0, mh1.distance(v21, v21), delta = 0.00001)
        self.assertAlmostEqual(0, mh2.distance(v31,v31), delta = 0.00001)
        self.assertAlmostEqual(0, mh3.distance(q1,q1), delta = 0.00001)


        mh1 = sdurw_math.WeightedManhattenMetricVector2D(sdurw_math.Vector2Dd(1.0,1.0))
        mh2 = sdurw_math.WeightedManhattenMetricVector3D(sdurw_math.Vector3Dd(1.0,1.0,1.0))
        mh3 = sdurw_math.WeightedManhattenMetricQ(sdurw_math.Q(3,1.0,1.0,1.0))

        self.assertAlmostEqual(0.1+0.2, mh1.distance(v21), delta = 0.00001)
        self.assertAlmostEqual(0.1+0.2+0.3, mh2.distance(v31), delta = 0.00001)
        self.assertAlmostEqual(0.1+0.2+0.3, mh3.distance(q1), delta = 0.00001)

        self.assertAlmostEqual(0, mh1.distance(v21, v21), delta = 0.00001)
        self.assertAlmostEqual(0, mh2.distance(v31,v31), delta = 0.00001)
        self.assertAlmostEqual(0, mh3.distance(q1,q1), delta = 0.00001)


        mh1 = sdurw_math.EuclideanMetricVector2D()
        mh2 = sdurw_math.EuclideanMetricVector3D()
        mh3 = sdurw_math.EuclideanMetricQ()

        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2), mh1.distance(v21), delta = 0.00001)
        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh2.distance(v31), delta = .00001)
        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh3.distance(q1), delta = 0.00001)

        self.assertAlmostEqual(0, mh1.distance(v21, v21), delta = 0.00001)
        self.assertAlmostEqual(0, mh2.distance(v31,v31), delta = 0.00001)
        self.assertAlmostEqual(0, mh3.distance(q1,q1), delta = 0.00001)


        mh1 = sdurw_math.WeightedEuclideanMetricVector2D(sdurw_math.Vector2Dd(1.0,1.0))
        mh2 = sdurw_math.WeightedEuclideanMetricVector3D(sdurw_math.Vector3Dd(1.0,1.0,1.0))
        mh3 = sdurw_math.WeightedEuclideanMetricQ(sdurw_math.Q(3,1.0,1.0,1.0))

        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2), mh1.distance(v21), delta = 0.00001)
        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh2.distance(v31), delta = 0.00001)
        self.assertAlmostEqual(math.sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh3.distance(q1), delta = 0.00001)

        self.assertAlmostEqual(0, mh1.distance(v21, v21), delta = 0.00001)
        self.assertAlmostEqual(0, mh2.distance(v31,v31), delta = 0.00001)
        self.assertAlmostEqual(0, mh3.distance(q1,q1), delta = 0.00001)


        mh1 = sdurw_math.InfinityMetricVector2D()
        mh2 = sdurw_math.InfinityMetricVector3D()
        mh3 = sdurw_math.InfinityMetricQ()

        self.assertAlmostEqual(0.2, mh1.distance(v21), delta = 0.00001)
        self.assertAlmostEqual(0.3, mh2.distance(v31), delta = 0.00001)
        self.assertAlmostEqual(0.3, mh3.distance(q1), delta = 0.00001)

        self.assertAlmostEqual(0, mh1.distance(v21, v21), delta = 0.00001)
        self.assertAlmostEqual(0, mh2.distance(v31,v31), delta = 0.00001)
        self.assertAlmostEqual(0, mh3.distance(q1,q1), delta = 0.00001)

    def test_Rotation2DTest(self):
        # String operations has been left out intentionally

        r1 = sdurw_math.Rotation2Dd().identity()
        v1 = sdurw_math.Vector2Dd(1,2)
        self.assertLess((v1 - r1 * v1).normInf(), 0.0000001)

        # Giver det mening at teste cast til integer ?
#        ri = int(sdurw_math.Rotation2Dd())

	# Rotation2D<int> ri;
	# ri = cast<int>(r1);
	# for (size_t i = 0; i < 2; i++)
	# 	for (size_t j = 0; j < 2; j++)
	# 		EXPECT_EQ((int)r1(i, j) , ri(i, j));
	# ri = rw::math::cast<int>(r1); // qualified lookup
	# for (size_t i = 0; i < 2; i++)
	# 	for (size_t j = 0; j < 2; j++)
	# 		EXPECT_EQ((int)r1(i, j) , ri(i, j));



        # Test comparison operators operator== and operator!= 
        rotcomp1 = sdurw_math.Rotation2Dd(1.1, -2.2, 3.3, 4.4)
        rotcomp2 = sdurw_math.Rotation2Dd(1.1, -2.2, 3.3, 4.4)
        self.assertEqual(rotcomp1, rotcomp2)
        self.assertTrue(not(rotcomp1 != rotcomp2))

        rotcomp3 = sdurw_math.Rotation2Dd(-1.1, 2.2, 3.3, 4.4)
        self.assertNotEqual(rotcomp1 , rotcomp3)
        self.assertTrue(not(rotcomp1 == rotcomp3))

    def test_Rotation3DTest(self):
        # String operations has been left out intentionally

        r1 = sdurw_math.Rotation3Dd().identity()
        v1 = sdurw_math.Vector3Dd(1,2,3)
        self.assertEqual((v1 - r1 * v1).normInf(), 0)

        eaa = sdurw_math.EAAd(sdurw_math.Vector3Dd(1.0, 0.0, 0.0),  math.pi / 2.0)
        r3 = eaa.toRotation3D()

        self.assertTrue(sdurw_math.LinearAlgebra.isSO(r3.e()))

        self.assertEqual(r3.e().rows() , r3.e().cols())
        self.assertEqual(r3.e().rows() , 3)

        self.assertEqual(r1 , r1)
        self.assertNotEqual(r1 , r3)

        ri = sdurw_math.castToFloat(r3)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(r3[i,j],ri[i,j],delta=1e-7)


    # Test comparison operators operator== and operator!=
        eaacomp1 = sdurw_math.EAAd(math.pi  / 2, 0, 0)
        rotcomp1 = eaacomp1.toRotation3D()

        eaacomp2 = sdurw_math.EAAd(math.pi  / 2, 0, 0)
        rotcomp2 = eaacomp2.toRotation3D()

        self.assertEqual(rotcomp1 , rotcomp2)
        self.assertTrue(not(rotcomp1 != rotcomp2))

        eaacomp3 = sdurw_math.EAAd(math.pi  / 4, 0, 0)
        rotcomp3 = eaacomp3.toRotation3D()
        self.assertNotEqual(rotcomp1 , rotcomp3)
        self.assertTrue(not(rotcomp1 == rotcomp3))

    def test_QTest(self):
        arr = [0.1, 0.2, 0.3]
        q1 = sdurw_math.Q(arr)
        self.assertEqual(q1.size() , 3)
        self.assertEqual(q1[2] , 0.3)
        self.assertEqual(q1[1] , 0.2)

        # stringstream operations has been left out intentionally

        q2 = sdurw_math.Q(q1)                                     # Make a copy of q1 using c++ copy constructor
        self.assertEqual(q2.size() , 3)
        self.assertAlmostEqual(q1[1], q2[1], delta = 0.00001)

        q3 = sdurw_math.Q(4,0.1,0.2,0.3,0.4)
        self.assertNotEqual(q3 , q1)
        q4 = sdurw_math.Q(3,0.1,0.2,0.4)
        self.assertNotEqual(q4 , q1)

        q34 = sdurw_math.concat(q3,q4)
        self.assertEqual(q34.size() , q3.size()+q4.size())
        self.assertEqual(q34[1] , q3[1])
        self.assertEqual(q34[q3.size()+1] , q4[1])

        dprod = sdurw_math.dot(q1,q1)
        self.assertAlmostEqual(dprod, q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2], delta = 0.00001)

        # testing constructors
        qc1 = sdurw_math.Q(1,0.1)
        qc2 = sdurw_math.Q(2,0.1,0.2)
        qc3 = sdurw_math.Q(3,0.1,0.2,0.2)
        qc4 = sdurw_math.Q(4,0.1,0.2,0.2,0.2)
        qc5 = sdurw_math.Q(5,0.1,0.2,0.2,0.2,0.2)
        qc6 = sdurw_math.Q(6,0.1,0.2,0.2,0.2,0.2,0.2)
        qc7 = sdurw_math.Q(7,0.1,0.2,0.2,0.2,0.2,0.2,0.2)
        qc8 = sdurw_math.Q(8,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2)
        qc9 = sdurw_math.Q(9,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2)

        # Test exception when creating Q with 11 arguments.
        with self.assertRaises(Exception): qc10_func()


if __name__ == '__main__':
    unittest.main()
