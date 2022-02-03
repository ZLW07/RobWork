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


class RPYTest(unittest.TestCase):

    def test_RPY(self):
        #Test default constructor
        rpy0 = sdurw_math.RPYd()

        self.assertEqual(rpy0[0] , 0.)
        self.assertEqual(rpy0[1] , 0.)
        self.assertEqual(rpy0[2] , 0.)

        # Test RPY(T alpha, T beta, T gamma) constructor
        rpy1 = sdurw_math.RPYd(0.1, 0.2, 0.3)
        self.assertEqual(rpy1[0] , 0.1)
        self.assertEqual(rpy1[1] , 0.2)
        self.assertEqual(rpy1[2] , 0.3)

        # Test toRotation3D and RPY(const Rotation3D<T>&) constructor
        rot3d = rpy1.toRotation3D()

        rpy2 = sdurw_math.RPYd(rot3d)
        self.assertLess(math.fabs(rpy2[0] - rpy1[0]), 1e-16)
        self.assertLess(math.fabs(rpy2[1] - rpy1[1]), 1e-16)
        self.assertLess(math.fabs(rpy2[2] - rpy1[2]), 1e-16)

        rpy1[0] = 0
        rpy1[1] = math.pi/2
        rpy1[2] = 0

        rot3d = rpy1.toRotation3D()
        self.assertEqual(rot3d[0,2] , 1)
        self.assertEqual(rot3d[1,1] , 1)
        self.assertEqual(rot3d[2,0] , -1)

        rpy3 = sdurw_math.RPYd(rot3d)
        self.assertEqual(rpy3[0] , rpy1[0])
        self.assertEqual(rpy3[1] , rpy1[1])
        self.assertEqual(rpy3[2] , rpy1[2])

        rpy1[0] = -2.5
        rpy1[1] = 0.4
        rpy1[2] = 4.2

        rpyf = sdurw_math.castToFloat(rpy1)
        for i in range (0,3):
            self.assertAlmostEqual(rpyf[i] , rpy1[i], delta = 1e-6)
        # qualified lookup would be same test in python

        # Test comparison operators operator== and operator!=
        comp1 = sdurw_math.RPYd(1.1, -2.2, 3.3)
        comp2 = sdurw_math.RPYd(1.1, -2.2, 3.3)
        self.assertEqual(comp1 , comp2)
        self.assertTrue(not (comp1 != comp2))
        comp3 = sdurw_math.RPYd(1.1, 2.2, -3.3)
        self.assertNotEqual(comp1 , comp3)
        self.assertTrue(not (comp1 == comp3))


if __name__ == '__main__':
    unittest.main() 