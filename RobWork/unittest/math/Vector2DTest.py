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
from sdurw_math.sdurw_math import normalize


class Vector2DTest(unittest.TestCase):

    def test_Vector2DTest_ADL(self):
        vd1 = sdurw_math.Vector2Dd(0.4, -0.7)
        vd2 = sdurw_math.Vector2Dd(0.9, 0.1)
        v5 = sdurw_math.Vector2Dd(3.0, 4.0)

        # Test Argument-Dependent Lookup (ADL)
        self.assertLess(math.fabs(sdurw_math.dot(vd1, vd2) - 0.29), 1e-15)

        cr = sdurw_math.cross(sdurw_math.Vector2Dd(2, 1), sdurw_math.Vector2Dd(1, 3))
        self.assertEqual(cr , 5)

        v5_norm = normalize(v5)
        self.assertLess(math.fabs(v5_norm.norm2() - 1) , 1e-15)


    def test_Vector2D(self):
        v1 = sdurw_math.Vector2Dd(1.0, 2.0)
        v2 = sdurw_math.Vector2Dd(v1)
        v3 = v1+v2
        v4 = sdurw_math.Vector2Dd(2.0, 4.0)

        self.assertEqual((v3-v4).normInf()  , 0)

        v5 = sdurw_math.Vector2Dd(3.0, 4.0)
        v5_norm = normalize(v5)
        self.assertLess(math.fabs(v5_norm.norm2() - 1) , 1e-15)
        v5_norm = sdurw_math.normalize(v5)  # qualified lookup
        self.assertLess(math.fabs(v5_norm.norm2() - 1) , 1e-15)
        l = v5.norm2()
        self.assertLess( math.fabs(v5_norm[0]-v5[0]/l), 1e-15)
        self.assertLess( math.fabs(v5_norm[1]-v5[1]/l), 1e-15)

        v5[0] = l
        v5[1] = 2*l
        self.assertEqual(v5[0] , l)
        self.assertEqual(v5[1] , 2*l)

         # String operations has been left out intentionally

        cr = sdurw_math.cross(sdurw_math.Vector2Dd(2.0, 1.0), sdurw_math.Vector2Dd(1.0, 3.0))   # qualified
        self.assertEqual(cr, 5)

        vd = sdurw_math.Vector2Dd(1.51, -2.51)
        vi = sdurw_math.castToFloat(vd)
        self.assertAlmostEqual(vi[0] , vd[0],delta=1e-7)
        self.assertAlmostEqual(vi[1] , vd[1],delta=1e-7)

        vd1 = sdurw_math.Vector2Dd(0.4, -0.7)
        vd2 = sdurw_math.Vector2Dd(0.9, 0.1)
        self.assertLess(math.fabs(sdurw_math.dot(vd1, vd2) - 0.29), 1e-15)

        # Test comparison operators operator== and operator!=
        comp1 = sdurw_math.Vector2Dd(1.1, -2.2)
        comp2 = sdurw_math.Vector2Dd(1.1, -2.2)
        self.assertEqual(comp1, comp2)
        self.assertTrue(not(comp1 != comp2))
        comp3 = sdurw_math.Vector2Dd(1.1, 2.2)
        self.assertNotEqual(comp1 , comp3)
        self.assertTrue(not(comp1 == comp3))


if __name__ == '__main__':
    unittest.main()
