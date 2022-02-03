***************************
Matrices, Vectors & Algebra
***************************

The rw::math namespace includes many math types for representing different types of vectors.
Math types in RobWork are based on :ref:`license_eigen`.

.. Matrix types:
   CameraMatrix
   InertiaMatrix
   Jacobian
   PerspectiveTransform2D
   Pose
   ProjectionMatrix
   Rotations
   Transforms

   Others:
   Line2D
   Line2DPolar
   Math::skew

   Vector types:
   Q
   Quaternion
   Rotation3DVector
   RPY
   VectorXX
   VelocityScrew6D
   Wrench6D

Notation
========

In general a diagonal notation form will be used to describe the relation
of vectors, rotation matrices, homogeneous transform, velocity screw,
and so on.

+------------------------------------------+-----------------------------------------------------------------+
| Expression                               | Description                                                     |
+==========================================+=================================================================+
| :math:`{}^{a}{\mathbf{P}}`               | Vector P seen in frame *a*                                      |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}{b}_{\mathbf{P}}`           | Translation of frame *b* seen in frame *a*                      |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}{b}_{\mathbf{R}}`           | Rotation of frame **b** seen in frame *a*                       |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}{b}_{\mathbf{T}}`           | Homogeneous transform of frame *b* seen in frame *a*            |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}_{b}{\mathbf{T}_v}^{c}_{d}` | | Velocity transform that transforms the reference frame from   |
|                                          | | *b* to *a* and the velocity reference point from *c* to *d*   |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}_{b}{\mathbf{T}_f}^{c}_{d}` | | Force transform that transforms the reference frame from      |
|                                          | | *b* to *a* and the force reference point from **c** to **d**  |
+------------------------------------------+-----------------------------------------------------------------+
| :math:`{}^{a}{b}_{\mathbf{J}}`           | A Jacobian matrix defined from reference frame *a* to frame *b* |
+------------------------------------------+-----------------------------------------------------------------+


When coordinate frames are visualized the axes are illustrated with the colors RGB, such that 
Red(x-axis), Green(y-axis) and Blue(z-axis).

Jacobians
===================

.. note::

   Documentation is being written...

Polynomials
===================

.. note::

   Documentation is being written...

Singular Value Decomposition (SVD)
==================================

.. note::

   Documentation is being written...

Linear Algebra
==================================

.. note::

   Documentation is being written...

.. pseudoInverse, determinant, inverse
   EigenDecomposition

Polynomials
==================================

.. note::

   Documentation is being written...