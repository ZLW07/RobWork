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


def norm_inf(vektor):
    print("\n vektor", vektor )
    vektor2 = sdurw_math.Vector3Dd(vektor)          # Make a copy of vektor using c++ copy constructor
    print("\n vektor2", vektor2 )
    return vektor2.normInf()


class Transform3DTest(unittest.TestCase):

    def test_Transform3D(self):
        d = sdurw_math.Vector3Dd(1, 2, 3)
        t = sdurw_math.Transform3Dd(d)               # Make a copy of d using c++ copy constructor
        v1 = sdurw_math.Vector3Dd(0, 0, 0)

        self.assertEqual(norm_inf(t * v1 - d) , 0)

        eaa = sdurw_math.EAAd(math.pi / 2, 0.0, 0.0)
        r1 = eaa.toRotation3D()
        t2 = sdurw_math.Transform3Dd(d, r1)
        t3 = t2 * sdurw_math.inverse(t2)
        xeaa = sdurw_math.EAAd(t3.R ())

        self.assertEqual(norm_inf(xeaa.axis()) , 0)
        self.assertEqual(xeaa.angle() , 0)
        self.assertLess(norm_inf(t3.P()) , 1e-15)

        tf = sdurw_math.castToFloat(t)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(tf[i,j],t[i,j],delta=1e-7)

        # Test comparison operators operator== and operator!= 
        eaacomp1 = sdurw_math.EAAd(math.pi / 2, 0.0, 0.0)
        rotcomp1 = eaacomp1.toRotation3D()
        comp1 = sdurw_math.Vector3Dd(1.1, -2.2, 3.3)
        tcomp1 = sdurw_math.Transform3Dd(comp1, rotcomp1)

        eaacomp2 = sdurw_math.EAAd(math.pi / 2, 0.0, 0.0)
        rotcomp2 = eaacomp2.toRotation3D()
        comp2 = sdurw_math.Vector3Dd(1.1, -2.2, 3.3)
        tcomp2 = sdurw_math.Transform3Dd(comp2, rotcomp2)

        self.assertEqual(tcomp1 , tcomp2)
        self.assertTrue(not(tcomp1 != tcomp2))

        eaacomp3 = sdurw_math.EAAd(math.pi / 4, 0.0, 0.0)
        rotcomp3 = eaacomp3.toRotation3D()
        comp3 = sdurw_math.Vector3Dd(1.1, -2.2, 3.3)
        tcomp3 = sdurw_math.Transform3Dd(comp3, rotcomp3)

        self.assertNotEqual(tcomp1 , tcomp3)
        self.assertTrue(not(tcomp1 == tcomp3))


if __name__ == '__main__':
    unittest.main()
