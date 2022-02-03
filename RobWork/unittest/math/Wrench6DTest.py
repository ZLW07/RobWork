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


class Wrench6DTest(unittest.TestCase):

    def test_Wrench6D_ADL(self):
        vec1 = sdurw_math.Vector3Dd(1.4, 2.5, 3.6)
        vec2 = sdurw_math.Vector3Dd(4.7, 5.8, 6.9)
        wrench = sdurw_math.Wrench6Dd(vec1, vec2)

        # Test Argument-Dependent Lookup (ADL)
        self.assertLess( math.fabs(wrench.norm1 () - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9)) , 1e-15)
        self.assertLess( math.fabs(wrench.norm2 () - math.sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9)) , 1e-15)
        self.assertLess( math.fabs(wrench.normInf () - 6.9) , 1e-15)


    def test_Wremch6D(self):
        # Verify that a default wrench contains 0 *
        wrench = sdurw_math.Wrench6Dd()

        self.assertEqual(wrench[0], 0)
        self.assertEqual(wrench[1], 0)
        self.assertEqual(wrench[2], 0)
        self.assertEqual(wrench[3], 0)
        self.assertEqual(wrench[4], 0)
        self.assertEqual(wrench[5], 0)


        # Verify that the wrench force and torque constructor appears to be working as intended
        # Verify that the force() and torque() members return the proper values
        vec1 = sdurw_math.Vector3Dd(1.4, 2.5, 3.6)
        vec2 = sdurw_math.Vector3Dd(4.7, 5.8, 6.9)
        wrench = sdurw_math.Wrench6Dd(vec1, vec2)

        self.assertEqual(wrench.force(), vec1)
        self.assertEqual(wrench.torque(), vec2)

        # Test unqualified lookup
        self.assertLess( math.fabs(wrench.norm1 () - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9)) , 1e-15)
        self.assertLess( math.fabs(wrench.norm2 () - math.sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9)) , 1e-15)
        self.assertLess( math.fabs(wrench.normInf () - 6.9) , 1e-15)

        # Test qualified lookup
        self.assertLess( math.fabs(sdurw_math.Wrench6Dd.norm1 (wrench) - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9)) , 1e-15)
        self.assertLess( math.fabs(sdurw_math.Wrench6Dd.norm2 (wrench) - math.sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9)) , 1e-15)
        self.assertLess( math.fabs(sdurw_math.Wrench6Dd.normInf (wrench) - 6.9) , 1e-15)


        vec1 = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        vec2 = sdurw_math.Vector3Dd(0.4, 0.5, 0.6)
        wrench = sdurw_math.Wrench6Dd(vec1, vec2)

        vsf = sdurw_math.castToFloat(wrench)

        for i in range (0,6):
            self.assertAlmostEqual(wrench[i], vsf[i], delta = 1e-6)

        # qualified lookup would be same test in python

        # Verify that the setForce() and setTorque() members appear to be working as intended
        vec1 = sdurw_math.Vector3Dd(4.1, 5.2, 6.3)
        vec2 = sdurw_math.Vector3Dd(7.4, 8.5, 9.6)
        wrench = sdurw_math.Wrench6Dd()

        wrench.setForce(vec1)
        wrench.setTorque(vec2)

        self.assertEqual(wrench.force(), vec1)
        self.assertEqual(wrench.torque(), vec2)

        eigen = wrench.e()

        for i in range (0,6):
            self.assertEqual(eigen[i,0] , wrench[i]) 


if __name__ == '__main__':
    unittest.main()