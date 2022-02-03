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


class Pose6DTest(unittest.TestCase):

    def test_Pose6D(self):
        p = sdurw_math.Pose6Dd(1.1,2.2,3.3,4.4,5.5,6.6)

        self.assertEqual(p[0] , 1.1)
        self.assertEqual(p[1] , 2.2)
        self.assertEqual(p[2] , 3.3)
        self.assertAlmostEqual(p[3], 4.4, delta = 1e-15)
        self.assertAlmostEqual(p[4], 5.5, delta = 1e-15)
        self.assertAlmostEqual(p[5], 6.6, delta = 1e-15)


        pf = sdurw_math.castToFloat(p)
        for i in range(6):
            self.assertAlmostEqual(pf[i],p[i],delta=1e-7)


if __name__ == '__main__':
    unittest.main()
