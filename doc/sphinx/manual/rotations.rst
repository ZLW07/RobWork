***************************
Rotations & Transformations
***************************

The most common representations for 3D rotations are implemented in RobWork.
In this page we will consider the `Rotation Matrix`_, `Axis-Angle (EAA)`_, `Roll Pitch Yaw Angle (RPY)`_, and `Quaternion`_ representations,
and how to convert between these representations. Finally, `Transformations`_ are considered.

Rotation Matrix
===================

A 3x3 rotation matrix can be constructed using the Rotation3D type.
The default Rotation3D<> type uses double precision, while Rotation3D<float> uses float precision.
Notice that it is only possible to specify the templated types in C++.
The equivalent types in the script interfaces (Python, Java and LUA) is Rotation3Dd and Rotation3Df
for double and float precision respectively.

The constructor for Rotation3D takes the elements of the rotation matrix
:math:`\mathbf{R} = \left[\begin{array}{ccc} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{array} \right]`

in the following order: Rotation3D(:math:`r_{11},r_{12},r_{13},r_{21},r_{22},r_{23},r_{31},r_{32},r_{33}`).

Code examples are shown below for different languages.

.. warning::

   There are two functions for finding the inverse rotation matrix, namely inverse(rot) and rot.inverse().
   The inverse(rot) returns a new rotation without altering the original rotation,
   while rot.inverse() calculates the inverse by changing the original rotation matrix.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex-rotation3d.cpp
   :language: c++
   :linenos:

API Reference: `rw::math::Rotation3D <../../apidoc/cpp/doxygen/classrw_1_1math_1_1Rotation3D.html>`__

See :ref:`interfaces_cpp` for more information about compilation and execution.

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex-rotation3d.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.Rotation3Dd` (double precision)
- :py:class:`rw.Rotation3Df` (float precision)

See :ref:`interfaces_python` for more information about execution.

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExRotation3D.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.Rotation3Dd (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/Rotation3Dd.html>`__
- `org.robwork.rw.Rotation3Df (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/Rotation3Df.html>`__

See :ref:`interfaces_java` for more information about compilation and execution.

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex-rotation3d.lua
   :language: lua
   :linenos:

See :ref:`interfaces_lua` for more information about execution of the script.

**Output**

.. code-block:: none

   Rotation double:
   Rotation3D(1, 0, 0, 0, 0, -1, 0, 1, 0)
   Rotation float:
   Rotation3D(1, 0, 0, 0, 0, -1, 0, 1, 0)
   Rotation inverse:
   Rotation3D(1, 0, 0, 0, 0, 1, 0, -1, 0)
   Identity:
   Rotation3D(1, 0, 0, 0, 1, 0, 0, 0, 1)

Axis-Angle (EAA)
===================

Equivalent Angle-Axis (EAA) is the RobWork type for rotations defined by the product of a unit vector and an angle of rotation.
The direction of the vector gives the axis of rotation, and the length of the vector gives the amount of rotation in radians.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex-eaa.cpp
   :language: c++
   :linenos:

API Reference: `rw::math::EAA <../../apidoc/cpp/doxygen/classrw_1_1math_1_1EAA.html>`__

See :ref:`interfaces_cpp` for more information about compilation and execution.

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex-eaa.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.EAAd` (double precision)
- :py:class:`rw.EAAf` (float precision)

See :ref:`interfaces_python` for more information about execution.

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExEAA.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.EAAd (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/EAAd.html>`__
- `org.robwork.rw.EAAf (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/EAAf.html>`__

See :ref:`interfaces_java` for more information about compilation and execution.

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex-eaa.lua
   :language: lua
   :linenos:

See :ref:`interfaces_lua` for more information about execution of the script.

**Output**

.. code-block:: none

   EAA:  EAA { 2.22144, 2.22144, 0}
    angle: 3.14159
    axis: Vector3D(0.707107, 0.707107, 0)
   Rotation from EAA: Rotation3D(2.22045e-16, 1, 8.65956e-17, 1, 2.22045e-16, -8.65956e-17, -8.65956e-17, 8.65956e-17, -1)
   Rotation: Rotation3D(-1, 0, 0, 0, 0, 1, 0, 1, 0)
   EAA from Rotation:  EAA { 0, 2.22144, 2.22144}
    angle: 3.14159
    axis: Vector3D(0, 0.707107, 0.707107)


Roll Pitch Yaw Angle (RPY)
===========================

Roll Pitch Yaw angles is one form of Euler angles where the rotation is defined by three consecutive rotations around the axes of a coordinate system.
In RobWork, RPY is the rotation around the z, y and x axes (in that order). Notice that the rotation around the y axis is the y axis after doing the rotation around z.
The rotation around the x axis is the x axis after doing the rotations around both z and y.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex-rpy.cpp
   :language: c++
   :linenos:

API Reference: `rw::math::RPY <../../apidoc/cpp/doxygen/classrw_1_1math_1_1RPY.html>`__

See :ref:`interfaces_cpp` for more information about compilation and execution.

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex-rpy.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.RPYd` (double precision)
- :py:class:`rw.RPYf` (float precision)

See :ref:`interfaces_python` for more information about execution.

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExRPY.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.RPYd (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/RPYd.html>`__
- `org.robwork.rw.RPYf (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/RPYf.html>`__

See :ref:`interfaces_java` for more information about compilation and execution.

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex-rpy.lua
   :language: lua
   :linenos:

See :ref:`interfaces_lua` for more information about execution of the script.

**Output**

.. code-block:: none

   RPY: RPY {3.14159, 1.5708, 0}
   Rotation from RPY: Rotation3D(-6.12323e-17, -1.22465e-16, -1, 7.4988e-33, -1, 1.22465e-16, -1, 0, 6.12323e-17)
   Rotation: Rotation3D(-1, 0, 0, 0, 0, 1, 0, 1, 0)
   RPY from Rotation: RPY {3.14159, -0, 1.5708}

Quaternion
===================

Quaternions are complex numbers, storing the rotation as 4 values.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex-quaternion.cpp
   :language: c++
   :linenos:

API Reference: `rw::math::Quaternion <../../apidoc/cpp/doxygen/classrw_1_1math_1_1Quaternion.html>`__

See :ref:`interfaces_cpp` for more information about compilation and execution.

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex-quaternion.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.Quaterniond` (double precision)
- :py:class:`rw.Quaternionf` (float precision)

See :ref:`interfaces_python` for more information about execution.

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExQuaternion.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.Quaterniond (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/Quaterniond.html>`__
- `org.robwork.rw.Quaternionf (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/Quaternionf.html>`__

See :ref:`interfaces_java` for more information about compilation and execution.

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex-quaternion.lua
   :language: lua
   :linenos:

See :ref:`interfaces_lua` for more information about execution of the script.

**Output**

.. code-block:: none

   Quaternion: Quaternion {0.707107, 0.707107, 0, 0}
   Rotation from Quaternion: Rotation3D(-2.22045e-16, 1, 0, 1, -2.22045e-16, 0, 0, 0, -1)
   Rotation: Rotation3D(-1, 0, 0, 0, 0, 1, 0, 1, 0)
   Quaternion from Rotation: Quaternion {0, 0.707107, 0.707107, 0}

Converting Rotations
====================

To convert between EAA, RPY and Quaternions, it is in general possible to convert the types via the full Rotation3D representation.
First convert to Rotation3D using the toRotation3D() function and then construct the new target type from this rotation matrix.

The RobWork Math class provides special functions for converting between EAA and Quaternion. Please see the quaternionToEAA and eaaToQuaternion functions.

API Reference:

- C++ : `rw::math::Math <../../apidoc/cpp/doxygen/classrw_1_1math_1_1Math.html>`__
- Python: :py:class:`rw.Math`
- Javadoc: `org.robwork.rw.Math <../../apidoc/java/javadoc/org/robwork/rw/Math.html>`__

Transformations
===============

The Transform3D type is a full 4x4 homogeneous transformation matrix.
A transformation matrix combines a rotation and a translation.
It can be constructed from a Vector3D and any of the rotation types above.

Alternatively, the Pose6D stores a pose using six values. Three values for the position and three values for the EAA orientation.

API Reference:

- C++ : `rw::math::Transform3D <../../apidoc/cpp/doxygen/classrw_1_1math_1_1Transform3D.html>`__
- Python: :py:class:`rw.Transform3Dd`
- Python: :py:class:`rw.Transform3Df`
- Python: :py:class:`rw.Pose6d`
- Python: :py:class:`rw.Pose6f`
- Javadoc: `org.robwork.rw.Transform3Dd <../../apidoc/java/javadoc/org/robwork/rw/Transform3Dd.html>`__
- Javadoc: `org.robwork.rw.Transform3Df <../../apidoc/java/javadoc/org/robwork/rw/Transform3Df.html>`__