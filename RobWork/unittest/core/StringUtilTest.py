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
 

import unittest                          # now we can use python unittest framework
import sys, os
import sdurw, sdurw_core                 # now we can use robwork python bindings


class TestStringUtil(unittest.TestCase):

    def test_GetRelativeDirectoryNameTest_HardcodedPathTest(self):
        # Get the current working directory
        current_working_directory = os.getcwd()

        # - incert "unittest" and remove "gtest/testfiles"
        test_directory = current_working_directory.replace("unittest", "gtest/testfiles")

        wcPath = test_directory + "workcells/simple_wc/SimpleWorkcell.wc.xml"
        robotGeometryTestString = test_directory + "workcells/simple_wc/PA10/Geometry/Rob-0"
        hardcodedResult = "PA10/Geometry/"
        result = sdurw_core.StringUtil.getRelativeDirectoryName(robotGeometryTestString, sdurw_core.StringUtil_getDirectoryName(wcPath))

        self.assertEqual(hardcodedResult, result)


if __name__ == '__main__':
    unittest.main() 