******************
Inverse Kinematics
******************

Inverse kinematics is the problem of calculating the configuration of a device, :math:`\mathbf{q}`,
such that the transformation from base to TCP frame becomes the desired,
:math:`\mathbf{T}_{base}^{TCP}(\mathbf{q}) = \mathbf{T}_{desired}`.

RobWork provides the InvKinSolver interface for solvers that are able to do this calculation.
The solvers can be grouped into two overall types:

- ClosedFormIK: solvers that can solve the inverse kinematics by analytical expressions.
- IterativeIK: solvers that uses multiple steps to iteratively find a solution. Typically these are using the device Jacobian to do so.

Both types of IK solvers take a transform (rw::math::Transform3D<>) as input and return configurations for a
device for which the transform from the base to the end of the device (rw::models::Device) equals the given transform.

In general the ClosedFormIK solvers can be expected to be computationally efficient,
but these solvers will typically only work for specific robots.
The iterative solvers can be expected to be less efficient, but are also applicable for more generic robots.

Closed Form Solvers
*******************

Specific closed form solvers are available for both Universal Robots and the Kuka IIWA robots.
The more generic PieperSolver is also available.

Universal Robots
==================

For Universal Robots, the closed form solver rw::invkin::ClosedFormIKSolverUR an be used.
This robot has 6 degrees of freedom, and the solver will provide up to 8 solutions.
Notice that for every joint there is a solution where the joint is rotated 360 degrees.
Than means that each of the 8 solutions can typically be achieved in 64 different ways. It is up to the user to add or subtract 360 degrees such that it suits the application best.

Kuka IIWA
==================

For the Kuka LBR IIWA 7 R800 robot, the closed form solver rw::invkin::ClosedFormIKSolverKukaIIWA an be used.
This robot has 7 degrees of freedom. It can have an infinite number of solutions, as the middle joint can be placed anywhere on a circle.
It is used very similarly to the `Universal Robots`_ solver, but the solve() function can optionally take an extra parameter.
The extra parameter is the direction of where to place the middle joint. The solutions returned is only deterministic when providing this extra argument.
Otherwise, a random direction is chosen. Up to 8 solutions is returned.

Pieper solver
==================

The rw::invkin::PieperSolver is a closed form solver that works for serial devices with 6 degrees of freedom revolute joints. It requires that the last three axes intersect.

Iterative Solvers
*******************

An iterative IK solver needs a start configuration from which to start
the iterative search. Depending on the start configuration and other
constraints, the IK solver may fail or succeed in finding a valid
configuration.

Jacobian Solver
==================

The rw::invkin::JacobianIKSolver is the basic iterative solver.
The method uses an Newton-Raphson iterative approach and is based on using the inverse of the device Jacobian to compute each local solution in each iteration.
Solving with the JacobianIKSolver is very similar to the closed form solvers, but remember that the result depend on the initial configuration of the robot.
The device configuration is stored in the state variable, and can be set by calling setQ on the device::

   device.setQ(q, state);

The JacobianIKSolver has methods for adjusting the step size and using different solver types. Please consult the API documentation for further information.

Meta Solvers
============

The solution of an iterative solver may depend on the starting configuration, provided in the
state given in the solve method. To search for all solution an iterative ik solver may 
be wrapped in the rw::invkin::IKMetaSolver which calls it a specified number of times with random 
start configurations. Many robots have ambiguities which can be resolved using the rw::invkin::AmbiguityResolver.

Parallel Robots
==================

For parallel robots, the rw::invkin::ParallelIKSolver can be used. It is an iterative solver somewhat similar to the JacobianIKSolver.
Where the JacobianIKSolver solves for only one objective, namely that the end-effector should be in a certain location, the ParallelIKSolver
simultaneously solves for the objective that the defined junctions must remain connected.
A stacked Jacobian is used to enforce these objectives simultaneously, and a Singular Value Decomposition (SVD) is used iteratively to find a solution.

Code Examples
*******************

In the following you find code examples on how to do inverse kinematics using the closed form solver for Universal Robots.
Other types of solvers are used in a very similar way.

C++
============

.. literalinclude:: ../../../RobWork/example/cpp/ex-invkin.cpp
   :language: c++
   :linenos:

In lines 22-30, the WorkCell is loaded. A SerialDevice is retrieved from the WorkCell in lines 31-33.
Always remember to check for null pointers.

In lines 35-36 the default state is retrieved from the WorkCell, and the closed form solver is constructed.

The desired transformation is defined in lines 38-39 and in line 40 the solutions are calculated by the solver.
The result is a vector of configurations, Q.

The result is printed in lines 42 to 49, along with some relevant information.

Python
============

.. literalinclude:: ../../../RobWork/example/python/ex-invkin.py
   :language: python
   :linenos:

In lines 7-13, the WorkCell is loaded. A SerialDevice is retrieved from the WorkCell in lines 14-16.
Always remember to check for null pointers.

In lines 18-19 the default state is retrieved from the WorkCell, and the closed form solver is constructed.

The desired transformation is defined in line 21 and in line 22 the solutions are calculated by the solver.
The result is a vector of configurations, Q.

The result is printed in lines 24 to 30, along with some relevant information.

.. note::

   Before Python can find the rw module, you must add the the RobWork/libs/BUILD_TYPE directory to the PYTHONPATH environment variable.
   
   Alternatively, you can import the sys module and call sys.path.append('RobWork/libs/BUILD_TYPE') before importing the rw module.

Java
============

.. literalinclude:: ../../../RobWork/example/java/src/ExInvKin.java
   :language: java
   :linenos:

In lines 13-21, the WorkCell is loaded. A SerialDevice is retrieved from the WorkCell in lines 22-24.
Always remember to check for null pointers.

In lines 26-28 the default state is retrieved from the WorkCell, and the closed form solver is constructed.

The desired transformation is defined in lines 30-31 and in line 32 the solutions are calculated by the solver.
The result is a vector of configurations, Q.

The result is printed in lines 34 to 41, along with some relevant information.

.. note::

   Before you can compile or run the program, you must add the RobWork/libs/BUILD_TYPE/rw_java.jar file to the classpath.
   This can be done by setting the CLASSPATH environment variable or calling java and javac with the -cp option.
   
   Java must be able to load the native JNI (Java Native Interface) library. This is what happens in line 11.
   Set the path with the option -Djava.library.path="RobWork/libs/BUILD_TYPE/" when calling java.
   In Linux, the path can also be added to the LD_LIBRARY_PATH environment variable.

LUA
============

.. literalinclude:: ../../../RobWork/example/lua/ex-invkin.lua
   :language: lua
   :linenos:

The rw module is imported in lines 1 and 2.
Normally, all classes and functions must be called with the module as a prefix, such as rw.WorkCellLoaderFactory.
To import all the names into the global namespace, the code in lines 2 can be used. Otherwise, you will have to use the "rw." prefix.

In lines 4-14, the WorkCell is loaded. A SerialDevice is retrieved from the WorkCell in lines 15-18.
Always remember to check for null pointers.

In lines 20-21 the default state is retrieved from the WorkCell, and the closed form solver is constructed.

The desired transformation is defined in line 18 and in line 19 the solutions are calculated by the solver.
The result is a vector of configurations, Q.

The result is printed in lines 26 to 34, along with some relevant information.

.. note::

   Before you can run the script, you must add the RobWork lua modules to the LUA_CPATH environment variable.
   
   In Linux this could for instance look like this (imports all rw modules):

   export LUA_CPATH="/path/to/RobWork/libs/release/Lua/?.so"

Output
------------------

The output from any of the programs above will be the following::

 Inverse Kinematics for UR10e.
  Base frame: UR10e.Base
  End/TCP frame: UR10e.Flange
  Target Transform: Transform3D(Vector3D(0.2, -0.2, 0.5), Rotation3D(-1, 0, 1.22465e-16, 0, 1, 0, -1.22465e-16, 0, -1))
 Found 8 solutions.
  Q[6]{-0.12278, 3.02845, 2.16872, -0.484776, -1.5708, -1.69358}
  Q[6]{-0.12278, -1.21998, -2.16872, 1.8179, -1.5708, -1.69358}
  Q[6]{-0.12278, -2.90011, 2.36859, 2.10231, 1.5708, 1.44802}
  Q[6]{-0.12278, -0.705404, -2.36859, -1.63839, 1.5708, 1.44802}
  Q[6]{1.69358, -2.43619, 2.36859, -1.5032, -1.5708, 0.12278}
  Q[6]{1.69358, -0.241482, -2.36859, 1.03928, -1.5708, 0.12278}
  Q[6]{1.69358, -1.92161, 2.16872, 1.3237, 1.5708, -3.01881}
  Q[6]{1.69358, 0.113143, -2.16872, -2.65682, 1.5708, -3.01881}
