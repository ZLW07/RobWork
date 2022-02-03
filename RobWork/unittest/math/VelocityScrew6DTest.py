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


class VelocetyScrew6DTest(unittest.TestCase):

    def test_VelocityScrew6DTest_ADL(self):
        linear1 = sdurw_math.Vector3Dd(1, 2, 3)
        angular1 = sdurw_math.EAAd(4, 5, 6)
        screw = sdurw_math.VelocityScrew6Dd(linear1, angular1)

        # Test Argument-Dependent Lookup (ADL)
        self.assertLess(math.fabs(screw.norm1() - (1 + 2 + 3 + 4 + 5 + 6)) , 1e-15)
        self.assertLess(math.fabs(screw.norm2() - math.sqrt(1.0 * 1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5 + 6 * 6)) , 1e-15)
        self.assertLess(math.fabs(screw.normInf() - 6) , 1e-15)


    def test_VelocetySrew6D(self):
        T = sdurw_math.Transform3Dd().identity()
        screw = sdurw_math.VelocityScrew6Dd(T)
        for i in range (0,6):
            self.assertEqual(screw[i] , 0)

        linear = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        angular = sdurw_math.EAAd(4, 5, 6)
        screw = sdurw_math.VelocityScrew6Dd(linear, angular)

        self.assertEqual(screw.linear()[0], linear[0])
        self.assertEqual(screw.linear()[1], linear[1])
        self.assertEqual(screw.linear()[2], linear[2])

        self.assertLess(math.fabs(screw[3] - angular.axis()[0] * angular.angle()), 1e-16)
        self.assertLess(math.fabs(screw[4] - angular.axis()[1] * angular.angle()), 1e-15)
        self.assertLess(math.fabs(screw[5] - angular.axis()[2] * angular.angle()), 1e-16)


        linear1 = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        angular1 = sdurw_math.EAAd(0.4, 0.5, 0.6)
        screw1 = sdurw_math.VelocityScrew6Dd(linear1, angular1)

        linear2 = sdurw_math.Vector3Dd(0.2, 0.3, 0.4)
        angular2 = sdurw_math.EAAd(0.5, 0.6, 0.7)
        screw2 = sdurw_math.VelocityScrew6Dd(linear2, angular2)

        screwA = sdurw_math.VelocityScrew6Dd(screw1)
        screwB = sdurw_math.VelocityScrew6Dd(screw2)

        screw1 += screw2
        for i in range (0,6):
            self.assertLess(math.fabs(screw1[i] - (screwA[i]+screwB[i])), 1e-15)

        screw2 -= screw1
        for i in range (0,6):
            self.assertLess(math.fabs(screw2[i] - -screwA[i]),  1e-15)


        linear1 = sdurw_math.Vector3Dd(1, 2, 3)
        angular1 = sdurw_math.EAAd(4, 5, 6)
        screw = sdurw_math.VelocityScrew6Dd(linear1, angular1)

        # Test unqualified lookup
        self.assertLess(math.fabs(screw.norm1() - (1 + 2 + 3 + 4 + 5 + 6)) , 1e-15)
        self.assertLess(math.fabs(screw.norm2() - math.sqrt(1.0 * 1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5 + 6 * 6)) , 1e-15)
        self.assertLess(math.fabs(screw.normInf() - 6) , 1e-15)
        
        # Test qualified lookup
        self.assertLess(math.fabs(sdurw_math.norm1(screw) - (1+2+3+4+5+6)),1e-15)
        self.assertLess(math.fabs(sdurw_math.norm2(screw) - math.sqrt(1.0 * 1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5 + 6 * 6)) , 1e-15)
        self.assertLess(math.fabs(sdurw_math.normInf(screw) - 6) , 1e-15)

        linear = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        angular = sdurw_math.EAAd(0.4, 0.5, 0.6)
        screw = sdurw_math.VelocityScrew6Dd(linear, angular)

        vsf = sdurw_math.castToFloat(screw)
        for i in range (0,6):
            self.assertAlmostEqual(screw[i] , vsf[i],delta=1e-5)

        linear = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        angular = sdurw_math.EAAd(0.4, 0.5, 0.6)
        screw = sdurw_math.VelocityScrew6Dd(linear, angular)
        eigen = screw.e()

        for i in range (0,6):
            self.assertAlmostEqual(eigen[i,0], screw[i],delta=1e-4) # TODO check after eigen 2 numpy works

        # Test comparison operators operator== and operator!= 
        linear1 = sdurw_math.Vector3Dd(1, 2, 3)
        angular1 = sdurw_math.EAAd(4, 5, 6)
        screw1 = sdurw_math.VelocityScrew6Dd(linear1, angular1)
        linear2 = sdurw_math.Vector3Dd(1, 2, 3)
        angular2 = sdurw_math.EAAd(4, 5, 6)
        screw2 = sdurw_math.VelocityScrew6Dd(linear2, angular2)
        self.assertEqual(screw1 , screw2)
        self.assertTrue(not(screw1 != screw2))

        linear3 = sdurw_math.Vector3Dd(1, 4, 3)
        angular3 = sdurw_math.EAAd(4, 5, 2)
        screw3 = sdurw_math.VelocityScrew6Dd(linear3, angular3)
        self.assertNotEqual(screw1 , screw3)
        self.assertTrue(not(screw1 == screw3))
    

if __name__ == '__main__':
    unittest.main()
