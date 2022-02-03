.. _interfaces_cpp:

*************
C++ Interface
*************

This collection of basic examples are provided to ease and aid the
introduction to programming applications using RobWork.

Template CMake Project
----------------------

This tutorial will demonstrate how to create a simple RobWork application using CMake.
First create a directory for your project.
Now add a **SampleTest.cpp** file and type in a small main application:

.. literalinclude:: ../../../RobWork/example/consoleapp/SampleTest.cpp
   :language: c++
   :linenos:
   :caption: RobWork/example/consoleapp/SampleTest.cpp

This example loads a WorkCell based on the argument given to the program.
In the first lines, the header files are included for the types we use in the program.
With the *using* statements we can avoid typing the full name when we use the Log, WorkCellLoader and WorkCell types in the code.
Alternatively, one would have to write rw::common::Log, rw::loaders::WorkCellLoader and rw::models::WorkCell respectively.
If you use more than one class from the same namespace, you should instead write:

.. code-block:: c++

   using namespace rw::common;
   using namespace rw::loaders;
   using namespace rw::models;

When many different namespaces and classes from RobWork are used, it can be somewhat tedious to write
the *using namespace* and includes in every file. Instead a general header file rw/rw.hpp and a macro can
be used to assemble all classes into one namespace. To include everything from RobWork We can instead write:

.. code-block:: c++

    #include <rw/rw.hpp>
    USE_ROBWORK_NAMESPACE
    using namespace robwork;

Notice that including all headers from RobWork can be quite heavy for the C++ preprocessor.
This might cause a penalty for the time used to compile the program.
For small programs like this, it is acceptable to import all the headers.
Always avoid importing rw/rw.hpp in header files.
Especially when developing libraries, as the header file might be included in many files that will each be hit by a compilation time penalty.  

Notice that *using* and *using namespace* must never be used in header files. Only if used inside functions.

In this example we write the output to Log::errorLog() and Log::infoLog().
For a console application like this, it makes little difference if you instead output directly to std::cout.
It is good practice to use the Log facility anyways.
If you later need to use a snippet of code or call a library function from a RobWorkStudio plugin,
the output will be shown in the RobWorkStudio Log plugin instead of the terminal.
In case RobWorkStudio is not launched from a terminal, this is very convenient. The output would otherwise be invisible to the user.

Now create a **CMakeLists.txt** file with the following content:

.. literalinclude:: ../../../RobWork/example/consoleapp/CMakeLists.txt
   :language: cmake
   :linenos:
   :caption: RobWork/example/consoleapp/CMakeLists.txt

Before running CMake, you should set the RW_ROOT environment variable to the path for RobWork.
CMake will complain if it does not find RobWork.

Now make a build directory and from this directory run the command::

   cmake -DCMAKE_BUILD_TYPE=Release path/to/project

On Windows you should also specify the generator based on your Visual Studio version::

   cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" path/to/project

You need to set the build type to the same build type that you used for compiling RobWork.
Insert the correct path to the project code (the directory with the CMakeLists.txt file).

Build the project with make (on Linux)::

   make

In Windows, open the ConsoleApp solution in Visual Studio (solution was generated with CMake).
Once the compilation succeeds, the executable will be put under the bin directory in the folder where CMakeLists.txt is located.

API documentation
-----------------

The user is encouraged to use the  `C++ API documentation <../../apidoc/cpp/doxygen>`_ to look up functionality, classes and functions in RobWork.
Please also consult the :ref:`manual` for various examples of using the RobWork library.
