.. _basic:

******************************
RobWork - A Programming Primer
******************************

This collection of basic tutorials and exercises are provided to ease and aid the
introduction to programming applications using RobWork.

Introduction
============

RobWork is a framework/library for simulation and control of robotics with emphasis
on industrial robotics and their applications.
The project was started in 2006 by Ph.D. students and master students at the Maersk Mc-Kinney Moeller
Institute, University of Southern Denmark.
Today the project is more mature and used by both researchers, Ph.D. students, master students and
robotics students. RobWork is mainly devided into 2 different parts. The basic
framework named RobWork consist of basic mathematics used for robots and algorithms — e.g.
motion planning and inverse kinematics.
The second part of the framework named RobWorkHardware contains drivers and code for
communicating with robots, cameras, canbus and others.
The major goal of the framework is to:

- Provide a single framework for offline and online robot programming including modelling, simulation and (realtime)control of robotics

The target users are:
 - Researchers who needs a common framework for experimental robotics
 - Students (you) who wants to experiment with the concepts of robotics
 - Implementers of robot applications

RobWork is currently being used for research and student exercises at the University of Southern Denmark.
You can read a lot more information at the RobWork website: http://www.robwork.dk
The following tutorials/exercises assume a functional installation of both RobWork and RobWorkStudio.
Please go to the homepage for installation tutorials.


My first cmake project
======================

This tutorial will demonstrate how to create a simple RobWork application using cmake.
First create a directory for your project:

.. code-block:: shell

   mkdir ~/workspace/first_project
   mkdir ~/workspace/first_project/build  ~/workspace/first_project/src
   cd ~/workspace/first_project

Create a “CMakeLists.txt” file in this directory ~/workspace/first_project and a “HelloWorld.cpp” in
the src directory (~/workspace/first_project/src). Insert the following cmake script into the CMakeLists.txt file::

    # Test CMake version
    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

    # The name of the project. (EDIT THIS)
    PROJECT(TutorialCollection)

    # Used to resolve absolute path names
    SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

    ##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
    # Now set the RW/RWS root (edit this if necessary)
    #
    # Set the RobWork root (edit in .bashrc if necessary)
    SET(RW_ROOT $ENV{RW_ROOT})                      # Use the environment variable  $RW_ROOT from .bashrc file (NOT necessary in PPA install)

    # set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)

    # Set build type to release
    SET(CMAKE_BUILD_TYPE Release)
    MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

    # Include default settings for constructing a robwork dependent project
    SET(RobWork_DIR ${RW_ROOT}/cmake)                # (NOT necessary in PPA install)
    FIND_PACKAGE(RobWork REQUIRED)
    LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

    # if you have additional libraries or include dirs then add them here
    INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )

    # And now we add any targets that we want
    add_executable(HelloWorld src/HelloWorld.cpp)
    target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES})

The script should be pretty much self explaining. However, it is important that the RW_ROOT is set to the
correct paths of robwork.
Now edit you “HelloWorld.cpp” file and type in a small main application.

.. code-block:: c++

   #include <rw/common/Log.hpp>
   using namespace rw::common;

   int main(int argc, char** argv) {
      Log::infoLog() << "Hey, we are printing to the RobWork log!\n";
      std::cout << "Which should just be standard out for now!" << std::endl;
   }

This is very basic but the reader should notice two important aspects. Firstly, the header file
rw/common/Log.hpp is included. This file includes only the Log class functionality which is the only class
used in the code. Secondly the “using namespace” clause is used such that we can call Log::infoLog()
instead of rw::common::Log::infoLog().
When many different namespaces and classes from robwork are used, it can be somewhat tedious to write
the “using namespace” and includes in every file. Instead a general header file rw/rw.hpp and a macro can
be used to assemble all classes into one namespace: “robwork”. We rewrite the code snippet from above.

.. code-block:: c++

   #include <rw/rw.hpp>
   
   USE_ROBWORK_NAMESPACE
   using namespace robwork;
   
   int main() {
       Log::infoLog() << "The using namespace enables us to call Log directly!\n";
       rw::common::Log::infoLog() << "We can still use the native namespace!\n";
       robwork::Log::infoLog() << "but also the general namespace!\n";
       return 0;
   }

Notice that when using this type friendly shortcut, the risk of name clashes between robwork classes and
other libraries become much higher. Also, NEVER use “using namespace” in headerfiles, unless its within a
function scope.

Compile and run the project
---------------------------
Do this to compile and run the project:

.. code-block:: shell

   cd ~/workspace/first_project/build
   cmake ..
   make
   ./HelloWorld

.. note::

   If making many changes, then it can be beneficial to delete the content of build directory before compiling again.


Math joggling
=============

This tutorial will demonstrate some of the basic math functionality available in RobWork. This is mostly
related to homogenous transformations, rotations, conversions and so on.
First add a new file “MathJoggling.cpp” to your cmake project from tutorial 1. Make sure that the file is
added as an executable in the end of the CMakeList.txt file.::

   # add another executable
   add_executable(MathJoggling src/MathJoggling.cpp)
   target_link_libraries(MathJoggling ${ROBWORK_LIBRARIES})

Add the standard static main code body in the “MathJoggling.cpp” and we are ready to play.

.. code-block:: c++

   #include <rw/rw.hpp>
   USE_ROBWORK_NAMESPACE
   using namespace robwork;
   int main(int argc, char** argv) {
   // main body, add your code here
   }

The main use of the math package is homogenous transformations, rotations, vectors. However, before
venturing into mathematical expressions we need to look at the different rotation representations. The
most user friendly format is probably euler angles where RobWork use a fixed axis ZYX euler representation
using the class “RPY” (Roll Pitch Yaw). The following snippet illustrates conversions between the rotation
formats.

.. code-block:: c++

   RPY<> rpy(0, 0, 90*Deg2Rad); // 90 degree rotation around x-axis
   Rotation3D<> rot = rpy.toRotation3D(); // create Rotation3D matrix
   EAA<> eaa( rot ); // construct eaa form rotation3d
   Quaternion<> quat( rot ); // construct quaternion from rotation3d
   // there are streaming operators for all math types
   Log::infoLog() << rpy << std::endl;
   Log::infoLog() << rot << std::endl;
   Log::infoLog() << eaa << std::endl;
   Log::infoLog() << quat << std::endl;

Operators are used throughout the math package to enable intuitive math expressions and all so streaming
as shown above. Now lets look at some of the most used functions in the math package.

.. code-block:: c++

   // rotate a vector (0,1,0) 90 degrees around x-axis
   Log::infoLog() << rot*Vector3D<>(0,1,0) << std::endl;
   // transform a vector
   Transform3D<> t1( Vector3D<>(0,0,1), rot);
   Log::infoLog() << t1*Vector3D<>(0,1,0) << std::endl;
   // calculate the inverse rotation
   Log::infoLog() << inverse( rot ) << std::endl;
   // calculate the inverse transform
   Log::infoLog() << inverse( t1 ) << std::endl;
   // get the rotation and translation part of a transform
   Log::infoLog() << t1.R() << t1.P() << std::endl;

Exercise 1 - Transformations
----------------------------
Try to set up two transformations T1 and T2. Set T1 with a position (x; y; z) = (1; 1; 1) and rotation (r;
p; y) = (90; 0; 0). Set T2 with a position (x; y; z) = (0; 0; 1) and rotation (r; p; y) = (0; 0; 0).

RobWork uses radians and not degrees. Take this into account.

Now calculate T1T2. What is the result? Construct a drawing of the frames (by hand)!


Exercise 2 – Point transform
----------------------------
Now consider that T1 and T2 are frames in a robotic system. T2 is described relative to T1 and T1 relative to
the world frame T0.

Using the two transformations from the previous exercise, calculate the position of pT2 = (0:5; 1; 0) with
respect to frame T1 and T0.

What is the result?