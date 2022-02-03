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
import sdurw_math, sdurw_core


class PropertyTypeMath(unittest.TestCase):

    def test_getType(self):

        value = sdurw_math.Vector3Dd()
        self.assertEqual(sdurw_core.PropertyType.Vector3D, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Vector2Dd()
        self.assertEqual(sdurw_core.PropertyType.Vector2D, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Q()
        self.assertEqual(sdurw_core.PropertyType.Q, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Transform3Dd()
        self.assertEqual(sdurw_core.PropertyType.Transform3D, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Rotation3Dd()
        self.assertEqual(sdurw_core.PropertyType.Rotation3D, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Rotation2Dd()
        self.assertEqual(sdurw_core.PropertyType.Rotation2D, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.RPYd()
        self.assertEqual(sdurw_core.PropertyType.RPY, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.EAAd()
        self.assertEqual(sdurw_core.PropertyType.EAA, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.Q()
        self.assertEqual(sdurw_core.PropertyType.Q, sdurw_core.PropertyType.getType(value).getId())

        value = sdurw_math.VelocityScrew6Dd()
        self.assertEqual(sdurw_core.PropertyType.VelocityScrew6D, sdurw_core.PropertyType.getType(value).getId())


if __name__ == '__main__':
    unittest.main()
