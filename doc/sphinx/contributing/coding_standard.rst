.. _coding_standard:

Coding Guidelines
=================

This section will present the coding guidelines that are used in RobWork.
We base the coding policy on the following guide lines:

http://geosoft.no/development/cppstyle.html

With the following exceptions:

- We name the C++ header files .hpp (instead of .h)
- We name the C++ source files .cpp (instead of .c++)
- We prefix member variables with _ (instead of using _ as a suffix)
- We use an indentation level of 4 characters
- We include RobWork headers before anything else

In-depth explanations follow.

Naming of source files:
***********************

Source files are named in UpperCamelCase (Java style) the following suffixes should be used:

- C++ header files: .hpp
- C++ source files: .cpp

As a rule of thumb: There should be one .hpp and one .cpp file for each class. The .hpp and .cpp
file should have the same name as the class

Include guards
**************

Use the following includeguards:

.. code-block:: cpp

   #ifndef RW_PACKAGENAME_CLASSNAME_HPP
   #define RW_PACKAGENAME_CLASSNAME_HPP
   ... // your code goes here
   #endif // RW_PACKAGENAME_CLASSNAME_HPP

example:

.. code-block:: cpp

   #ifndef RW_MODELS_SERIALDEVICE_HPP
   #define RW_MODELS_SERIALDEVICE_HPP
   ...
   #endif // RW_MODELS_SERIALDEVICE_HPP

Use of namespaces
*****************

.. code-block:: cpp

   namespace rw {
       namespace packagename {

       }
   }

Avoid using "using namespace" in .hpp files. This violates the principle of namespaces.
"using namespace" is only allowed in .cpp files.

Class definitions
*****************

Place public members before protected members, place protected members before private members

.. code-block:: cpp

   class SerialDevice
   {
       public:
           void someFunction();

       protected:
           ...

       private:
           ...
   };

Documentation
*************

We use doxygen for documentations, doxygen tags should start with a "\@" (JavaDoc style). Brief member
descriptions should be prefixed by \@brief

We use the same writing style as used in the Java API (see http://java.sun.com/j2se/1.5.0/docs/api/)

Example of good brief member descriptions:

- \@brief Removes all of the elements of this collection
- \@brief Constructs an ActionEvent object
- \@brief Returns a parameter string identifying this action event

Example of bad brief member descriptions:

- This method is used for finding the square root

There should be a space between class members and the next documentation block

Right:

.. code-block:: cpp

   class Test {
       public:
           // @brief Performs the first test
           void test1();

           // @brief Performs the second test
           void test2();
   };

Wrong:

.. code-block:: cpp

   class Test {
       public:
           // @brief Performs the first test
           void test1();
           // @brief Performs the second test
           void test2();
   };

Indentation
***********

We use indentation level 4. Please be careful to setup your IDE to use spaces and not tabs.

Notation for math
*****************

When possible use the following notation for code and documentation:

========================================= =========================================== =============================================== ====== ==============================
Documentation                             Doxygen                                     Sphinx                                          Code   Example of use
========================================= =========================================== =============================================== ====== ==============================
:math:`\thetak`                           \\f$\\thetak\\f$                            \:math\:\`\\thetak\`                            thetak Angle-axis (EAA).
:math:`\robax{a}{\mathbf{p}}`             \\f$\\robax{a}{\\mathbf{p}}\\f$             \:math\:\`\\robax{a}{\\mathbf{p}}\`             aP     Point with respect to frame a.
:math:`\robabx{a}{b}{\mathbf{T}}`         \\f$\\robabx{a}{b}{\\mathbf{T}}\\f$         \:math\:\`\\robabx{a}{b}{\\mathbf{T}}\`         aTb    Transform a to b (or b wrt. a)
:math:`\robabcdx{a}{b}{c}{d}{\mathbf{J}}` \\f$\\robabcdx{a}{b}{c}{d}{\\mathbf{J}}\\f$ \:math\:\`\\robabcdx{a}{b}{c}{d}{\\mathbf{J}}\` aJb
**x**                                     \\b x  **or**  \\f$\\mathbf{x}\\f$          \*\*x\*\*  **or**  \:math\:\`\\mathbf{x}\`      x      Pose
**d**                                     \\b d  **or**  \\f$\\mathbf{d}\\f$          \*\*d\*\*  **or**  \:math\:\`\\mathbf{d}\`      d      Vector
:math:`\mathbf{\nu}`                      \\f$\\mathbf{\\nu}\\f$                      \:math\:\`\\mathbf{\\nu}\`                      V      VelocityScrew
**v**                                     \\b v  **or**  \\f$\\mathbf{v}\\f$          \*\*v\*\*  **or**  \:math\:\`\\mathbf{v}\`      v      Linear velocity
:math:`\mathbf{\omega}`                   \\f$\\mathbf{\\omega}\\f$                   \:math\:\`\\mathbf{\\omega}\`                   w      Angular velocity
**q**                                     \\b q  **or**  \\f$\\mathbf{q}\\f$          \*\*q\*\*  **or**  \:math\:\`\\mathbf{q}\`      q      Joint configuration
========================================= =========================================== =============================================== ====== ==============================

Notice that the following macros are special macros defined for use in the RobWork documentation:

======== =======================================================
Command  TeX definition
======== =======================================================
thetak   \\newcommand{\\thetak}{\\theta\\mathbf{\\hat{k}}}
robax    \\newcommand{\\robax}[2]{{}^{#1}{#2}}
robabx   \\newcommand{\\robabx}[3]{{}^{#1}{#3}_{#2}}
robabcdx \\newcommand{\\robabcdx}[5]{{}^{#1}_{#2}{#5}^{#3}_{#4}}
======== =======================================================

Include files
*************

.hpp files should be included in the follwing order:

- (for .cpp files only) ClassName.hpp
- .hpp files from same namespace
- RobWork .hpp files
- ext includes
- other includes
- boost includes
- stl includes

Example.: (SerialDevice.cpp)

.. code-block:: cpp

   #include "SerialDevice.hpp"

   #include "DependentJoint.hpp"
   #include "Joint.hpp"

   #include <rw/kinematics/Frame.hpp>
   #include <rw/kinematics/Kinematics.hpp>

   #include <vector>

Feel free to add spaces to indicate the include groups as shown above. Sort the files in each group lexicographically.

For source files in test, example and demo use the above rules but include the
RobWork files as library files instead of local files (for instance use <rw/models/Joint.hpp> instead of "Joint.hpp")

Try to reduce .hpp dependencies
*******************************

Try to reduce .hpp dependencies by not including more .hpp files than absolutely necessary.
Use forward declarations when possible.

Use tests
*********

Do not remove or comment-out tests from the test directory. When you add new classes or functions, be sure to create a test of it.
New tests should be written based on the Google Test framework, while older ones are written as Boost tests.

Use the RobWork smart pointer
*****************************

All classes which are expected to be passed as pointers should declare a pointer typedef using the
RobWork smart pointer rw::common::Ptr.

.. code-block:: cpp

   class MyClass;

   // A pointer to a MyClass
   typedef rw::common::Ptr<MyClass> MyClassPtr;

Classes taking pointers to objects should likewise use the smart pointer to determine ownership
and avoid memory leaks.

.. note::
   We are currently considering to directly use the std::smart_ptr available in C++11 instead of the RobWork smart pointer.

Templates
===============

To combine all of the best practices described here, an example of a .hpp and .cpp file is provided.
These can also be used at templates when developing new classes.

.hpp file
*********

.. code-block:: cpp

   /********************************************************************************
    * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
    * Faculty of Engineering, University of Southern Denmark 
    * 
    * Licensed under the Apache License, Version 2.0 (the "License");
    * you may not use this file except in compliance with the License.
    * You may obtain a copy of the License at
    *
    *     http://www.apache.org/licenses/LICENSE-2.0
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    ********************************************************************************/

   #ifndef RW_MODELS_SERIALDEVICE_HPP
   #define RW_MODELS_SERIALDEVICE_HPP

   /**
    * @file SerialDevice.hpp
    */

   #include "JointDevice.hpp"

   #include <vector>

   namespace rw {
       namespace models {
           /** @addtogroup models */
           //! @{

           /**
            * @brief The device for a serial chain.
            *
            * SerialChain is like JointDevice except that SerialChain has the
            * additional guarantee that the joints lie on a single parent to child
            * path of the kinematic tree.
            */
           class SerialDevice : public JointDevice
           {
               public:
                   //! @brief smart pointer type to this class
                   typedef rw::common::Ptr<SerialDevice> Ptr;
                   //! @brief smart pointer type to this const class
                   typedef rw::common::Ptr< const SerialDevice > CPtr;

                   /**
                    * @brief Constructor
                    *
                    * @param first [in] the base frame of the robot
                    * @param last [in] the end-effector of the robot
                    * @param name [in] name of device
                    * @param state [in] the connectedness of the frames
                    */
                   SerialDevice(
                           kinematics::Frame* first,
                           kinematics::Frame* last,
                           const std::string& name,
                           const kinematics::State& state);

                   //! @brief destructor
                   virtual ~SerialDevice() {}

                   /**
                    * @brief Frames of the device.
                    *
                    * This method is being used when displaying the kinematic
                    * structure of devices in RobWorkStudio. The method really
                    * isn't of much use for everyday programming.
                    *
                    * @return list of raw Frame pointers.
                    */
                   const std::vector<kinematics::Frame*>& frames() const;

                   ...

               private:
                   std::vector<kinematics::Frame*> _kinematicChain;
           };
           //! @}
       } // end models namespace
   } // end rw namespace

   #endif // end include guard

.cpp file
*********

.. code-block:: cpp

   /********************************************************************************
    * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
    * Faculty of Engineering, University of Southern Denmark 
    * 
    * Licensed under the Apache License, Version 2.0 (the "License");
    * you may not use this file except in compliance with the License.
    * You may obtain a copy of the License at
    *
    *     http://www.apache.org/licenses/LICENSE-2.0
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    ********************************************************************************/

   #include "SerialDevice.hpp"

   #include "DependentJoint.hpp"
   #include "Joint.hpp"

   #include <rw/kinematics/Frame.hpp>
   #include <rw/kinematics/Kinematics.hpp>

   #include <vector>

   using namespace rw::common;
   using namespace rw::kinematics;
   using namespace rw::math;
   using namespace rw::models;

   namespace
   {
       std::vector<Joint*> getJointsFromFrames(const std::vector<Frame*>& frames)
       {
           std::vector<Joint*> active;
           ...
           return active;
       }
   }

   SerialDevice::SerialDevice(Frame* first,
                              Frame* last,
                              const std::string& name,
                              const State& state):
       JointDevice(name, first,
               last,getJointsFromFrames(...),state),
       _kinematicChain(getChain(first, last, state))
   {
   }

   const std::vector<Frame*>& SerialDevice::frames() const
   {
       return _kinematicChain;
   }