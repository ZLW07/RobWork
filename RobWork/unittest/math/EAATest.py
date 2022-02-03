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
import numpy as np


class EAA(unittest.TestCase):

    def test_ADL(self):
        vec1 = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        eaa2 = sdurw_math.EAAd(0.4, 0.5, 0.6)
        c = sdurw_math.cross(vec1, eaa2)
        self.assertLess(math.fabs(c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15)
        self.assertLess(math.fabs(c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15)
        self.assertLess(math.fabs(c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15)

    def test_CastTest(self):
        eaacast = sdurw_math.EAAd(0.1, 0.2, 0.3)
        eaaf = sdurw_math.castToFloat(eaacast)
        for i in range(0,3,1):
            self.assertAlmostEqual(eaaf[i], float(eaacast[i]), 7)
        eaaf = sdurw_math.castToFloat(eaacast)    # qualified lookup
        for i in range(0,3,1):
            self.assertAlmostEqual(eaaf[i], float(eaacast[i]), 7)

    def test_CrossProductTest(self):
        vec1 = sdurw_math.Vector3Dd(0.1, 0.2, 0.3)
        eaa2 = sdurw_math.EAAd(0.4, 0.5, 0.6)
        c = sdurw_math.cross(vec1, eaa2)
        self.assertLess(math.fabs(c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15)
        self.assertLess(math.fabs(c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15)
        self.assertLess(math.fabs(c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15)

        c = sdurw_math.cross(vec1, eaa2)    # qualified lookup
        self.assertLess(math.fabs(c[0] - (0.2 * 0.6 - 0.3 * 0.5)), 1e-15)
        self.assertLess(math.fabs(c[1] + (0.1 * 0.6 - 0.3 * 0.4)), 1e-15)
        self.assertLess(math.fabs(c[2] - (0.1 * 0.5 - 0.4 * 0.2)), 1e-15)

    def test_MiscTest(self):
        # 0 degree
        e0 = sdurw_math.EAAd(0.0, 0.0, 0.0)
        self.assertEqual(e0.angle () , 0)
        self.assertEqual((e0.axis ()).normInf() , 0)
        xe0 = sdurw_math.EAAd(e0.toRotation3D())
        self.assertEqual(xe0.angle() , e0.angle())
        self.assertEqual((xe0.axis () - e0.axis ()).normInf() , 0)

        # 180 degree
        e180_1 = sdurw_math.EAAd(math.pi, 0.0, 0.0)
        self.assertLess(math.fabs(e180_1.angle () - math.pi), 1e-16)
        self.assertEqual((e180_1.axis () - sdurw_math.Vector3Dd (1.0, 0.0, 0.0)).normInf() , 0)
        xe180_1 = sdurw_math.EAAd(e180_1.toRotation3D())
        self.assertLess(math.fabs (xe180_1.angle () - e180_1.angle ()) , 1e-16)
        self.assertEqual((xe180_1.axis () - e180_1.axis ()).normInf() , 0)

        e180_2 = sdurw_math.EAAd(0.0, math.pi, 0.0)
        self.assertEqual(e180_2.angle (), math.pi)
        self.assertEqual((e180_2.axis () - sdurw_math.Vector3Dd (0.0, 1.0, 0.0)).normInf (), 0)
        xe180_2 = sdurw_math.EAAd(e180_2.toRotation3D())
        self.assertEqual(xe180_2.angle (), e180_2.angle ())
        self.assertEqual((xe180_2.axis () - e180_2.axis ()).normInf(), 0)

        e180_3 = sdurw_math.EAAd(0.0, 0.0, math.pi)
        self.assertEqual(e180_3.angle (), math.pi)
        self.assertEqual((e180_3.axis () - sdurw_math.Vector3Dd (0.0, 0.0, 1.0)).normInf (), 0)
        xe180_3 = sdurw_math.EAAd(e180_3.toRotation3D())
        self.assertEqual(xe180_3.angle (), e180_3.angle ())
        self.assertEqual((xe180_3.axis () - e180_3.axis ()).normInf(), 0)

        val1 = 0.3
        val2 = 0.4
        val3 = 0.5

        for sign1 in [-1,1]:
            for sign2 in [-1,1]:
                for sign3 in [-1,1]:
                    axisInput = sdurw_math.Vector3Dd(sign1 * val1, sign2 * val2, sign3 * val3)
                    axisInput = axisInput.normalize()
                    e180 = sdurw_math.EAAd(axisInput * math.pi)

                    self.assertAlmostEqual(e180.angle (), math.pi, 15)

                    e180axis = e180.axis ()
                    self.assertAlmostEqual((e180axis - axisInput).normInf (), 0.0, delta = 1e-15)
                    xe180 = sdurw_math.EAAd(e180.toRotation3D ())
                    self.assertAlmostEqual(xe180.angle (), e180.angle (), delta = 1e-13)
                    xe180axis = xe180.axis ()
                    # vector can point both ways and still be valid
                    if ((xe180axis - e180axis).norm2 () > (-xe180axis - e180axis).norm2 ()) :
                        xe180axis = -xe180axis
                    self.assertAlmostEqual((xe180axis - e180axis).normInf (), 0.0, delta = 1e-15)

        # Testing different sign combinations for 180 degree rotations - epsilon
        eps = 1e-7;    # should be less than the hardcoded threshold in EAA.cpp
        for sign1 in [-1,1]:
            for sign2 in [-1,1]:
                for sign3 in [-1,1]:
                    axisInput = sdurw_math.Vector3Dd(sign1 * val1, sign2 * val2, sign3 * val3)
                    axisInput = axisInput.normalize()
                    e180 = sdurw_math.EAAd(axisInput * (math.pi - eps))
                    self.assertAlmostEqual(e180.angle (), math.pi - eps, delta = 1e-13)
                    e180axis = e180.axis ()
                    self.assertAlmostEqual((e180axis - axisInput).normInf (), 0.0, delta = 1e-15)
                    xe180 = sdurw_math.EAAd(e180.toRotation3D ())
                    self.assertAlmostEqual(xe180.angle (), e180.angle (), 13)
                    xe180axis = xe180.axis ()
                    self.assertAlmostEqual((xe180axis - e180axis).normInf (), 0.0, delta = 1e-7)

        # Testing different sign combinations for 180 degree rotations + epsilon
        for sign1 in [-1,1]:
            for sign2 in [-1,1]:
                for sign3 in [-1,1]:
                    axisInput = sdurw_math.Vector3Dd(sign1 * val1, sign2 * val2, sign3 * val3)
                    axisInput = axisInput.normalize()
                    e180 = sdurw_math.EAAd(axisInput * (math.pi + eps))
                    self.assertAlmostEqual(e180.angle (), math.pi + eps, delta = 1e-13)
                    e180axis = e180.axis ()
                    self.assertAlmostEqual((e180axis - axisInput).normInf (), 0.0, delta = 1e-15)
                    xe180 = sdurw_math.EAAd(e180.toRotation3D ())
                    self.assertAlmostEqual(xe180.angle (), math.pi - eps, 13)           # should choose angle < Pi
                    xe180axis = xe180.axis ()
                    self.assertAlmostEqual((xe180axis + e180axis).normInf (), 0.0, delta = 1e-7)   # should flip vector to get angle < Pi

       #  90 degree's around x axis
        v1 = sdurw_math.Vector3Dd(1.0, 0.0, 0.0)
        e1 = sdurw_math.EAAd(v1,  math.pi / 2.0)
        self.assertEqual(e1.angle (), math.pi / 2.0)
        self.assertEqual((e1.axis () - v1).normInf (), 0)
        rot = e1.toRotation3D ()
        e2 = sdurw_math.EAAd(rot)      # Make a copy of obj1 using c++ copy constructor
        self.assertEqual((e1.axis () - e2.axis ()).normInf (), 0)
        self.assertEqual(e1.angle (), e2.angle ())

        # Test comparison operators operator== and operator!= 
        comp1 = sdurw_math.EAAd(1.1, -2.2, 3.3)
        comp2 = sdurw_math.EAAd(1.1, -2.2, 3.3)
        self.assertTrue(comp1 == comp2)
        self.assertFalse(comp1 != comp2)
        comp3 = sdurw_math.EAAd(1.1, -2.2, 3.3)
        self.assertTrue(comp1 == comp3)
        self.assertFalse(comp1 != comp3)

         # Test different "problematic setups which previously has resulted in NaN errors

        t0 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000), 
                                     sdurw_math.Rotation3Dd(0.9990961814,
                                                            -0.0003891806,
                                                            -0.0425144844,
                                                            0.0004163897,
                                                            0.9999992493,
                                                            0.0006269312,
                                                            0.0425143000,
                                                            -0.0006440670,
                                                            0.9990960000))
        t1 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000), 
                                     sdurw_math.Rotation3Dd(-0.0000000000,
                                                            -0.0000000000,
                                                            1.0000000000,
                                                            0.0000000000,
                                                            -1.0000000000,
                                                            -0.0000000000,
                                                            1.0000000000,
                                                            0.0000000000,
                                                            0.0000000000))
        r0 = (sdurw_math.inverse (t0) * t1).R () 
        r0.normalize()
        eaa0 = sdurw_math.EAAd(r0)
        self.assertFalse(sdurw_math.Math.isNaN (eaa0.angle ()))

        t2 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(-1.0000000000,
                                                            -0.0000000000,
                                                            0.0000000000,
                                                            0.0000000000,
                                                            -1.0000000000,
                                                            0.0000000000,
                                                            -0.0000000000,
                                                            0.0000000000,
                                                            1.0000000000))
        r1 = (sdurw_math.inverse (t0) * t2).R ()
        r1.normalize()
        eaa1 = sdurw_math.EAAd(r1)
        self.assertFalse(sdurw_math.Math.isNaN (eaa1.angle ()))

        t3 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(1.0000000000,        
                                                            -0.0000000000,
                                                            -0.0000000000,
                                                            -0.0000000000,
                                                            -1.0000000000,
                                                            0.0000000000,
                                                            -0.0000000000,
                                                            -0.0000000000,
                                                            -1.0000000000))
        r2 = (sdurw_math.inverse (t0) * t3).R ()
        r2.normalize()
        eaa2 = sdurw_math.EAAd(r2)
        self.assertFalse(sdurw_math.Math.isNaN (eaa2.angle ()))

        t4 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(-0.9993793864,
                                                            -0.0352195084,
                                                            -0.0004122380,
                                                            -0.0004294999,
                                                            0.0004964944,
                                                            0.9999993429,
                                                            -0.0352193000,
                                                            0.9993790000,
                                                            -0.0005113220))
        t5 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(0.0000000000,
                                                            -1.0000000000,
                                                            -0.0000000000,
                                                            0.0000000000,
                                                            0.0000000000,
                                                            -1.0000000000,
                                                            1.0000000000,
                                                            0.0000000000,
                                                            0.0000000000))
        r3 = (sdurw_math.inverse (t4) * t5).R ()
        r3.normalize()
        eaa3 = sdurw_math.EAAd(r3)
        self.assertFalse(sdurw_math.Math.isNaN (eaa3.angle ()))

        t6 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(0.0000000000,
                                                            1.0000000000,
                                                            0.0000000000,
                                                            0.0000000000,
                                                            0.0000000000,
                                                            -1.0000000000,
                                                            -1.0000000000,
                                                            0.0000000000,
                                                            0.0000000000))
        r4 = (sdurw_math.inverse (t4) * t6).R ()
        r4.normalize()
        eaa4 = sdurw_math.EAAd(r4)
        self.assertFalse(sdurw_math.Math.isNaN (eaa4.angle ()))

        t7 = sdurw_math.Transform3Dd(sdurw_math.Vector3Dd(0.0004406670, 0.1325120000, 0.0236778000),
                                     sdurw_math.Rotation3Dd(-1.0000000000,
                                                            0.0000000000,
                                                            0.0000000000,
                                                            -0.0000000000,
                                                            0.0000000000,
                                                            -1.0000000000,
                                                            -0.0000000000,
                                                            -1.0000000000,
                                                            -0.0000000000))
        r5 = (sdurw_math.inverse (t4) * t7).R ()
        r5.normalize()
        eaa5 = sdurw_math.EAAd(r5)
        self.assertFalse(sdurw_math.Math.isNaN (eaa5.angle ()))

    def test_scalarOperatorTest(self):
        obj = sdurw_math.EAAd(3, 3, 3)

        test1 = obj.elemMultiply(2)
        test2 = obj.elemDivide(2)
        test3 = obj.elemAdd(2)
        test4 = obj.elemSubtract(2)

        for i in range(obj.size()):
            self.assertEqual(test1[i], 6.0)
            self.assertEqual(test2[i], 3.0 / 2.0)
            self.assertEqual(test3[i], 5.0)
            self.assertEqual(test4[i], 1.0)

    def test_Vector3DOperatorTest(self):
        obj1 = sdurw_math.EAAd(3, 3, 3)
        obj2 = sdurw_math.Vector3Dd(2, 2, 2)

        test1 = obj1.elemMultiply(obj2)
        test2 = obj1.elemDivide(obj2)
        test3 = obj1 + obj2
        test3x = obj2 + obj1
        test4 = obj1 - obj2
        test4x = obj2 - obj1
        test7 = sdurw_math.EAAd(obj1)      # Make a copy of obj1 using c++ copy constructor
        test7 += obj2
        test8 = sdurw_math.EAAd(obj1)      # Make a copy of obj1 using c++ copy constructor
        test8 -= obj2

        for i in range(0, obj1.size(), 1):
            self.assertEqual(test1[i], 6.0)
            self.assertEqual(test2[i], 3.0 / 2.0)
            self.assertEqual(test3[i], 5.0)
            self.assertEqual(test3x[i], 5.0)
            self.assertEqual(test4[i], 1.0)
            self.assertEqual(test4x[i], -1.0)
            self.assertEqual(test7[i], 5.0)
            self.assertEqual(test8[i], 1.0)

        self.assertNotEqual(obj1, obj2)
        self.assertNotEqual(obj2, obj1)
        self.assertFalse(obj1 == obj2)
        self.assertFalse(obj2 == obj1)
        obj2 = obj1.toVector3D()
        self.assertTrue(obj1 == obj2)
        self.assertTrue(obj2 == obj1)
        self.assertFalse(obj1 != obj2)
        self.assertFalse(obj2 != obj1)

    def test_EAAOperatorTest(self):
        obj1 = sdurw_math.EAAd(3, 3, 3)
        obj2 = sdurw_math.EAAd(2, 2, 2)

        test1 = obj1 * obj2
        test2 = obj1.elemMultiply(obj2)
        test3 = obj1.elemDivide(obj2)
        test4 = obj1.elemAdd(obj2)
        test5 = obj1.elemSubtract(obj2)

        for i in range(0, obj1.size(), 1):
            self.assertAlmostEqual(test1[i], 1.3724012715315643, delta = 1e-10)
            self.assertEqual(test2[i], 6.0)
            self.assertEqual(test3[i], 3.0/2.0)
            self.assertEqual(test4[i], 5.0)
            self.assertEqual(test5[i], 1.0)

        comp1 = sdurw_math.EAAd(1.1, -2.2, 3.3)
        comp2 = sdurw_math.EAAd(comp1)       # Make a copy of comp1 using c++ copy constructor
        comp3 = -sdurw_math.EAAd(comp1)      # Make a copy of comp1 using c++ copy constructor
        self.assertEqual(comp1, comp2)
        self.assertFalse(comp1 != comp2)
        self.assertTrue(comp1 != comp3)
        self.assertFalse(comp1 == comp3)

    def test_EigenOperatorTest(self):
        obj1 = sdurw_math.EAAd(3, 3, 3)
        obj2 = np.array([2, 2, 2])           # This how we define Eigen in python using NumPy arrays 

#  TODO Awaits instantiating of NumPy object 
#
#        test1  = obj1.elemMultiply(obj2)
#        test2  = obj1.elemDivide(obj2)
#        test3  = obj1 + obj2
#        test3x = obj2 + obj1
#        test4  = obj1 - obj2
#        test4x = obj2 - obj1
#        test7 = obj1
#        test7 += obj2
#        test8 = obj1
#        test8 -= obj2
#
#        for i in range(0, obj1.size(), 1):
#            self.assertEqual(test1[i], 6.0)
#            self.assertEqual(test2[i], 3.0 / 2.0
#            self.assertEqual(test3[i], 5.0
#            self.assertEqual(test3x[i], 5.0)
#            self.assertEqual(test4[i], 1.0)
#            self.assertEqual(test4x[i], -1.0)
#            self.assertEqual(test7[i], 5.0
#            self.assertEqual(test8[i], 1.0)

#        self.assertTrue(obj1 != obj2)
#        self.assertTrue(obj2 != obj1)
#        self.assertFalse(obj1 == obj2)
#        self.assertFalse(obj2 == obj1)
#        obj2 = obj1.e()
#        self.assertTrue(obj1 == obj2)
#        self.assertTrue(obj2 == obj1)
#        self.assertFalse(obj1 != obj2)
#        self.assertFalse(obj2 != obj1)

    def test_MathOperators(self):
        obj1 = sdurw_math.EAAd(1, 2, 3)
        obj2 = sdurw_math.EAAd(3, 2, 1)
        obj3 = sdurw_math.Vector3Dd(1,2,3)
        obj4 = sdurw_math.Vector3Dd(3,2,1)

        test1 = obj1.cross(obj2)
        test2 = obj1.dot(obj2)
        test3 = obj1.cross(obj4)
        test4 = obj1.dot(obj4)
        test5 = obj3.cross(obj2.toVector3D())
        test6 = obj3.dot(obj2.toVector3D())

# TODO check numpy array conversion in swig
#        self.assertEqual(test1, np.cross(obj1.e (),obj2.e ()))        # AttributeError: 'EigenVector3d' object has no attribute 'cross'
#        self.assertEqual(test2,np.cross(obj1.e (),obj2.e ()))         # AttributeError: 'EigenVector3d' object has no attribute 'dot'


        self.assertEqual(test3, test1)
        self.assertEqual(test4, test2)
        self.assertEqual(test5, test1)
        self.assertEqual(test6, test2)


if __name__ == '__main__':
    unittest.main() 