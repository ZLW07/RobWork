**********
Kinematics
**********

The basic building blocks of a WorkCell are Frames. These are ordered in a tree-structure where the root
node always is the **WORLD** frame. All frames has a parent which their position and orientation is relative to.
Descending in this tree accumulating frame transformations is basically forward kinematics. The *Kinematics*
class is a utility class for calculating forward kinematics.

Forward Kinematics
==================

The examples below show how to calculate the transformation between two frames in the WorkCell.
The final lines shows how a DAF frame can be attached to another frame. In this case the mframe is attached to the end frame of a serial device.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_fwd-kinematics.cpp
   :language: c++
   :linenos:

See :ref:`interfaces_cpp` for more information about compilation and execution.

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_fwd_kinematics.py
   :language: python
   :linenos:

See :ref:`interfaces_python` for more information about execution.

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExFwdKinematics.java
   :language: java
   :linenos:

See :ref:`interfaces_java` for more information about compilation and execution.

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_fwd-kinematics.lua
   :language: lua
   :linenos:

See :ref:`interfaces_lua` for more information about execution of the script.

Device Kinematics
=================

The device class also define utility functions for calculating forward kinematics, at least those that relate to
the device. Additionally the Device has functionality to compute the Device Jacobian, setting and getting
the joint configurations and getting the joint limits.

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_fwd-kinematics-device.cpp
   :language: c++
   :linenos:

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_fwd_kinematics_device.py
   :language: python
   :linenos:

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExFwdKinematicsDevice.java
   :language: java
   :linenos:

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_fwd-kinematics-device.lua
   :language: lua
   :linenos:

Kinematics trees and states
===========================

The kinematic structure of the work cell is represented by a tree of
frames (see rw::kinematics::Frame). The root of the kinematic tree is
called the *world frame* (rw::models::WorkCell::getWorldFrame()).
Each frame has a transformation (see rw::math::Transform3D) relative
to its parent frame and this transformation may change in response to
values assigned for the frame. A revolute joint of a device (see
rw::models::RevoluteJoint) is for example implemented as a frame that
has a single value that rotates the frame relative to its parent. 
Besides revolute joints RobWork also supports prismatic joints and dependent
joints for which the value may depend on other joints.

It is important in RobWork to note that the values for the frames are
not stored *within* the frames, but are instead stored explicitly in
a value of type rw::kinematics::State. Given a state for the workcell,
the transform of a frame relative to its parent can be calculated with
rw::kinematics::Frame::getTransform().

The frames of the workcell are always organized in a tree, but for
certain frames the parent that they are connected to can dynamically
be changed. These frames are called *dynamically attachable frames* or
*DAFs* for short. The parent that a DAF is attached to is
not stored within the DAF itself, but is instead stored externally in
a State value. Different state values can thus correspond to different
structures of the tree. Given a state for the workcell the parent and
children of a frame can be retrieved with
rw::kinematics::Frame::getParent() and
rw::kinematics::Frame::getChildren().

Because the values of the frames and the attachments of DAFs are
stored outside of the workcell, we say that the workcell is *stateless*.
This enables a workcell and the associated data to be used 
concurrently in multiple threads as well as to easily communicate the 
entire state of a workcell.

To illustrate these important ideas, this example shows how to print
the structure of the kinematic tree of the workcell and for each frame
print also the position of the frame in space:

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_print-kinematic-tree.cpp
   :language: c++
   :linenos:

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_print_kinematic_tree.py
   :language: python
   :linenos:

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExPrintKinematicTree.java
   :language: java
   :linenos:

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_print-kinematic-tree.lua
   :language: lua
   :linenos:

We see from this example that given a state, it is straight-forward to
compute the transform of every single frame in the workcell. RobWork
has some utilities to make calculation of forward kinematics
convenient in the day to day work, such a rw::kinematics::FKTable and rw::kinematics::FKRange described below.

World transforms for a set of frames
====================================

rw::kinematics::FKTable computes the forward kinematics for a number
of frames for a common state. The results of the forward kinematics
are stored in the FKTable object so that the transform for a frame is
not computed over and over again. This example shows how the transform
for a sequence of frames can be efficiently computed:

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_world-transforms.cpp
   :language: c++
   :linenos:

API Reference: `rw::kinematics::FKTable <../../apidoc/cpp/doxygen/classrw_1_1kinematics_1_1FKTable.html>`__

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_world_transforms.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.FKTable`

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExWorldTransforms.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.FKTable (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/FKTable.html>`__

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_world-transforms.lua
   :language: lua
   :linenos:

Relative transforms for a pair of frames
======================================== 

rw::kinematics::FKRange computes the relative transform for a pair of
frames. To efficiently compute the relative transform for a pair of
frames the path in the kinematic tree that connects the frames must be
computed. Knowing the path and the relative transform between adjacent
frames of the path (rw::kinematics::Frame::getTransform()) the full
transform from start to end of the path can be computed. This example
shows the use of rw::kinematics::FKRange:

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_frame-to-frame-transform.cpp
   :language: c++
   :linenos:

API Reference: `rw::kinematics::FKRange <../../apidoc/cpp/doxygen/classrw_1_1kinematics_1_1FKRange.html>`__

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_frame_to_frame_transform.py
   :language: python
   :linenos:

API Reference:

- :py:class:`rw.FKRange`

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExFrameToFrameTransform.java
   :language: java
   :linenos:

API Reference:

- `org.robwork.rw.FKRange (Javadoc) <../../apidoc/java/javadoc/org/robwork/rw/FKRange.html>`__

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_frame-to-frame-transform.lua
   :language: lua
   :linenos:

If you repeatedly compute the forward kinematics for the same pair of
frames and the same parent-child structure of the tree, you can reuse
the rw::kinematics::FKRange object so that e.g. the path connecting
the frames need not be recomputed. For example, given a pair of frames
and a set of states the relative transforms that relate the frames can
be computed efficiently as follows:

**C++**

.. literalinclude:: ../../../RobWork/example/cpp/ex_frame-to-frame-transforms.cpp
   :language: c++
   :linenos:

**Python**

.. literalinclude:: ../../../RobWork/example/python/ex_frame_to_frame_transforms.py
   :language: python
   :linenos:

**Java**

.. literalinclude:: ../../../RobWork/example/java/src/ExFrameToFrameTransforms.java
   :language: java
   :linenos:

**LUA**

.. literalinclude:: ../../../RobWork/example/lua/ex_frame-to-frame-transforms.lua
   :language: lua
   :linenos:

The frameToFrameTransform() utility function is available as
rw::kinematics::Kinematics::frameTframe().

Dynamically attachable frames and movable frames
================================================

A *dynamically attachable frame* (DAF) is a frame for which the
parent frame can be changed. We say that the frame is attached to a
new parent frame (rw::kinematics::Frame::attachFrame()). A DAF can be
attached to any frame of the workcell except itself. You should avoid
attaching a DAF to a child of its subtree as this will create a cycle
in the kinematic structure. Frames of any type can be a DAF. You can
check if a frame is a DAF like this:

.. literalinclude:: ../../../RobWork/example/snippets/ex-is-daf.cpp
   :language: c++
   :linenos:

DAFs are used for example to simulate the picking up of an item by a
gripper. The item is represented by a DAF and is initially attached to
some frame of the workcell. When the gripper is closed, the picking up
of the item is simulated by attaching the item frame to the gripper
frame.

If the parent frame of a DAF is changed, the world transform of the
DAF will generally change also. When simulating the picking up of an
item, you do not want the item to instantly change position in space.
Therefore a DAF is often a *movable frame*
(rw::kinematics::MovableFrame) also. A movable frame is a frame for
which an arbitrary transform can be set for the transform of the frame
relative to its parent (rw::kinematics::MovableFrame::setTransform()).
To simulate the gripping of the item, the parent of the frame is set
to the frame of the gripper, and at the same time the relative
transform of the item is assigned a value that equals the transform
from gripper to item. This procedure is carried out as follows:

.. literalinclude:: ../../../RobWork/example/snippets/ex-grip-frame.cpp
   :language: c++
   :linenos:

The function receives the current state of the workcell as input and
updates this state to reflect the gripping of the item. Recall that
the frames themselves are stateless: The attachment of the DAF and its
change of relative transform is stored entirely within the state.

RobWork provides utilities for the above in the form of the
rw::kinematics::Kinematics::gripFrame() and
rw::kinematics::Kinematics::gripMovableFrame() collection of
functions.