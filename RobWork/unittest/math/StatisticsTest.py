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


class Statistics(unittest.TestCase):

    def test_Statistics(self):
        stats = sdurw_math.Statistics()
        stats.add(1.1)
        stats.add(0.2)
        stats.add(-0.065)
        stats.add(0.005)
        self.assertEqual(0.31,stats.mean())
        self.assertAlmostEqual(0.28995,stats.variance(), delta = 0.00001)

        self.assertEqual(0.31,stats.meanAndVariance()[0])
        self.assertAlmostEqual(0.28995,stats.meanAndVariance()[1], delta = 0.00001)
        self.assertAlmostEqual(0.1025,stats.median(), delta = 0.00001)

        self.assertEqual(-0.065,stats.minValue())
        self.assertEqual(1.1,stats.maxValue())

        self.assertEqual(-0.065,stats.minAndMaxValue()[0])
        self.assertEqual(1.1,stats.minAndMaxValue()[1])

        self.assertAlmostEqual(0.2915800123437472,stats.angularMean(), delta = 0.00001)
        self.assertAlmostEqual(0.2904023945936753,stats.angularVariance(), delta = 0.00001)

        self.assertAlmostEqual(0.2915800123437472,stats.angularMeanAndVariance()[0], delta = 0.00001)
        self.assertAlmostEqual(0.2904023945936753,stats.angularMeanAndVariance()[1], delta = 0.00001)
        
        
    def test_StatisticsFloat(self):
        stats = sdurw_math.Statistics_f()
        stats.add(1.1)
        stats.add(0.2)
        stats.add(-0.065)
        stats.add(0.005)
        self.assertAlmostEqual(0.31,stats.mean(), delta = 0.00001)
        self.assertAlmostEqual(0.28995,stats.variance(), delta = 0.00001)

        self.assertAlmostEqual(0.31,stats.meanAndVariance()[0], delta = 0.00001)
        self.assertAlmostEqual(0.28995,stats.meanAndVariance()[1], delta = 0.00001)
        self.assertAlmostEqual(0.1025,stats.median(), delta = 0.00001)

        self.assertAlmostEqual(-0.065,stats.minValue(), delta = 0.00001)
        self.assertAlmostEqual(1.1,stats.maxValue(), delta = 0.00001)

        self.assertAlmostEqual(-0.065,stats.minAndMaxValue()[0], delta = 0.00001)
        self.assertAlmostEqual(1.1,stats.minAndMaxValue()[1], delta = 0.00001)

        self.assertAlmostEqual(0.2915800123437472,stats.angularMean(), delta = 0.00001)
        self.assertAlmostEqual(0.2904023945936753,stats.angularVariance(), delta = 0.00001)

        self.assertAlmostEqual(0.2915800123437472,stats.angularMeanAndVariance()[0], delta = 0.00001)
        self.assertAlmostEqual(0.2904023945936753,stats.angularMeanAndVariance()[1], delta = 0.00001)


if __name__ == '__main__':
    unittest.main()
