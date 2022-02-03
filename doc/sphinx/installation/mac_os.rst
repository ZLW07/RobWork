MacOS compilation
**********************

RobWork can be built by the user.
This guide shows the steps for doing this in macOS High Sierra v. 10.13.

If you have any suggestions or additions to the guide, please post it on the issue
tracker https://gitlab.com/sdurobotics/RobWork/issues . This guide was
last revised in April 2020.

RobWork is basically multiple projects:

- RobWork :
  is the core part including math, kinematics, planning and so on.
- RobWorkStudio :
  is the GUI which enable visualization and more user friendly interfaces through gui plugins
- RobWorkSim :
  is an extension to the RobWork core functionality which adds dynamic simulation of bodies,
  devices and several tactile sensors.
- RobWorkHardware :
  is mostly drivers (with RobWork datatypes) for common hardware,
  or hardware on which RobWork platforms have been built eg. SDH, cameras,
  CAN-devices, the Universal robot arm, serial port...

Note that RobWork is needed to run RobWorkStudio, RobWorkSim and
RobWorkHardware. Therefore it is not possible to use these, without
having RobWork installed on the machine.

Installing dependencies
=======================

RobWork depends on third-party software that must be installed prior to
compilation. This includes both build tools and third-party libraries.
In Linux it is quite easy to set up the dependencies as these are
available as packages in the systems package manager.

The commands shown in the following must be run from a terminal.

To install most dependencies homebrew is used.

.. code-block:: shell

    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"

Build Tools
-----------

To be able to checkout code it is necessary to install some source code
management (SCM) tools.
To be able to checkout the code from our own Git repository, a Git client is
needed.

.. code-block:: shell

    brew install git

To compile the C++ code, CMake must be used to prepare RobWork for compilation.

.. code-block:: shell

    brew install cmake

RobWork Required Dependencies
-----------------------------

Boost and eigen are the only required dependency for RobWork

.. code-block:: shell

    brew install boost eigen

RobWork Recommended additional Dependencies
-------------------------------------------

These dependencies are not necessary for running RobWork, 
but will add extra functionality to RobWork.

.. code-block:: shell

    brew install assimp fcl pkg-config libomp

RobWork Optional Dependencies
-----------------------------

Xerces can be used some places in RobWork for opening XML files. It is
no longer a strict requirement, as RobWork is now able to use a Boost
parser instead. If you enable compilation of non-standard parts of
RobWork, or need to compile old RobWork-dependent projects, it might be
a good idea to compile Xerces:

.. code-block:: shell

    brew install xerces-c

SWIG (optional) is a tool that makes it possible to generate a LUA
script interface for RobWork. SWIG must be version 3 or newer (since we use C++11).
Python and Java interfaces are also possible, but require that Python or Java
SDK is installed as well. All of these interfaces can be generated if
you install the following packages:

.. code-block:: shell

  brew install swig lua boost-python3 python openjdk

to use Java you need to link it to the system installed packages with:

.. code-block:: shell 

  sudo ln -sfn $(brew --prefix)/opt/openjdk/libexec/openjdk.jdk /Library/Java/JavaVirtualMachines/openjdk.jdk


RobWorkStudio Required Dependencies
-----------------------------------

RobWorkStudio requires Qt to be installed. Only Qt5 is supported:

.. code-block:: shell

    brew install qt5

RobWorkSim Dependencies
-----------------------------------
If you need to do dynamic simulations, you will probably need the RobWorkSim package. 
If you are in doubt and just need RobWorkStudio, you can likely skip this.

Open Dynamics Engine (ODE) can be installed through the package manager:

.. code-block:: shell

    brew install ode

Bullet Physics can also be installed through the package manager.

.. code-block:: shell

    brew install bullet

Building RobWork
================

When the dependencies are installed, go ahead and download the newest
version of RobWork from the Git repository at:

https://gitlab.com/sdurobotics/RobWork

In the terminal, this is done as follows: (be sure that you are located
in the directory where you want to install RobWork) In this guide we will install in the home directory

.. code-block:: shell

    cd ~
    git clone https://gitlab.com/sdurobotics/RobWork.git
    cd RobWork

.. note::

   In order to access the repository, you will need to have an account at GitLab.com and follow the procedure here to gain access: http://robwork.dk/getaccess

Setup CMake Options & Environment
---------------------------------

Before running CMake to build RobWork, some environment variables might
need to be set. This is generally not needed when installing
dependencies through the package manager. If one or more dependencies
were compiled manually, one must be careful that CMake actually finds
the dependency correctly. A good advice before building RobWork, is to
actually read the CMake output carefully. Running CMake will be
discussed later, but the CMake output will typically reveal early in the
process if a dependency was not found. Building RobWork can take quite
some time, and it is a petty building everything, just to discover that
some functionality was disabled due to a unmet dependency (especially a
problem for the optional dependencies).

There are overall two methods to let RobWork know where a dependency is
installed. One is to set an environment variable, another is to set
CMake options when running the CMake command. Environment variables can
be set up one time for all in the users home folder in the .bash_profile file,
while CMake options has to be specified each time you need to rebuild
RobWork from scratch. The later does however give more fine-grained
control, as it allows multiple versions of dependencies to be installed
on the system. The version to use is then selected explicitly when
running CMake.

In :ref:`CMake Options & Environment<cmake-options>`: we try to
give an overview of the correct variables to set for the various
dependencies.

Compiling RobWork
-----------------

In the following it is assumed that RobWork was checked out to the
folder ~/RobWork, and that this is the current directory. Add a build
directory for each of the projects you want to build:

.. code-block:: shell

    mkdir Build
    mkdir Build/RW
    mkdir Build/RWStudio
    mkdir Build/RWSim
    mkdir Build/RWHardware

Now we are ready to build RobWork. Run CMake in the newly created build
directory for RobWork, and run make afterwards to build the project:

.. code-block:: shell

    cd ~/RobWork/Build/RW
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWork
    make -j4

Look carefully through the CMake output before running the make command.
Check that there is no errors, and that the required dependencies are
correctly found. The -j4 argument to make will build RobWork on 4 CPU
cores. Note that you need at least 1 GB of memory per thread when
building. Ie. building with 4 cores requires around 4 GB of RAM.

For RobWorkStudio:

.. code-block:: shell

    cd ~/RobWork/Build/RWStudio
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkStudio
    make -j4

For RobWorkSim:

.. code-block:: shell

    cd ~/RobWork/Build/RWSim
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkSim
    make -j4

For RobWorkHardware:

.. code-block:: shell

    cd ~/RobWork/Build/RWHardware
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkHardware
    make -j4

Finally, we need to add the following paths to ~/.bash_profile:

.. code:: shell

    #ROBWORK#
    export RW_ROOT=~/RobWork/RobWork/
    export RWS_ROOT=~/RobWork/RobWorkStudio/
    export RWHW_ROOT=~/RobWork/RobWorkHardware/
    export RWSIM_ROOT=~/RobWork/RobWorkSim/

Remember to only add paths to the components you have actually
installed. Ie. if you only installed RobWork and RobWorkStudio, the
paths for RobWorkSim and RobWorkHardware should not be set.

By setting these environment variables, it will be possible for other
projects to find the RobWork projects.
