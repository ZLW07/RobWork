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


def close_enough(q1, q2):
    if math.fabs((q1).getQx () - (q2).getQx ()) < 1e-16 and \
       math.fabs((q1).getQy () - (q2).getQy ()) < 1e-16 and \
       math.fabs((q1).getQz () - (q2).getQz ()) < 1e-16 and \
       math.fabs((q1).getQw () - (q2).getQw ()) < 1e-16 :
        return True
    
    return False


class QuaternionTest(unittest.TestCase):

    def test_Conversion(self):
        print("\n Mangler")


    def test_MiscTest(self):
        # Test Quaternion(T a, T b, T c, T d) constructor
        q1 = sdurw_math.Quaternion(1.0, 2.0, 3.0, 4.0)

        self.assertEqual(q1.getQx (), 1.0)
        self.assertEqual(q1.getQy (), 2.0)
        self.assertEqual(q1.getQz (), 3.0)
        self.assertEqual(q1.getQw (), 4.0)


        # Test toRotation3D and Quaternion(const Rotation3D&) constructor
        r1 = q1.toRotation3D()
        q2 = sdurw_math.Quaternion(r1)

        # Test arithmetic functions
        a = sdurw_math.Quaternion(1, 2, 3, 4)
        b = sdurw_math.Quaternion(4, 3, 2, 1)
        c = sdurw_math.Quaternion(0, 0, 0, 0)
        d = sdurw_math.Quaternion(0, 0, 0, 0)

        self.assertTrue(close_enough(a, sdurw_math.Quaternion(1.0, 2.0, 3.0, 4.0)))
        self.assertTrue(close_enough(+a, sdurw_math.Quaternion(1.0, 2.0, 3.0, 4.0)))
        self.assertTrue(close_enough(-a, sdurw_math.Quaternion(-1.0, -2.0, -3.0, -4.0)))
        
        self.assertTrue(close_enough((a + b), sdurw_math.Quaternion(5.0, 5.0, 5.0, 5.0)))
        self.assertTrue(close_enough((a - b), sdurw_math.Quaternion(-3, -1, 1, 3)))

#        self.assertTrue(close_enough((a += b), sdurw_math.Quaternion(5, 5, 5, 5)))
#        self.assertTrue(close_enough((a -= b), sdurw_math.Quaternion(1, 2, 3, 4)))
        h = sdurw_math.Quaternion(a)
#        self.assertTrue(close_enough((h *= b, a * b))

        self.assertEqual(a, sdurw_math.Quaternion(1, 2, 3, 4))
        self.assertTrue( not(a != sdurw_math.Quaternion(1, 2, 3, 4)))
        self.assertNotEqual(a, sdurw_math.Quaternion(0, 2, 3, 4))
        self.assertNotEqual(a, sdurw_math.Quaternion(1, 0, 3, 4))
        self.assertNotEqual(a, sdurw_math.Quaternion(1, 2, 0, 4))
        self.assertNotEqual(a, sdurw_math.Quaternion(1, 2, 3, 0))

        af = sdurw_math.castToFloat(a)
        for i in range(4):
            self.assertAlmostEqual(a[i],af[i],delta=1e-7)


if __name__ == '__main__':
    unittest.main()
