Ubuntu compilation
**********************

RobWork can be built by the user.
This guide shows the steps for doing this in Ubuntu 16.04, 18.04, 20.04 and 20.10.
The compilation on these platforms are tested continuously.
Current status of the pipeline for the RobWork master branch is:

.. image:: https://robwork.dk/getBadge
   :target: https://gitlab.com/sdurobotics/RobWork

If you have any suggestions or additions to the guide, please post it on the issue
tracker https://gitlab.com/sdurobotics/RobWork/issues . This guide was
last revised in November 2020.

.. note::

   Our goal is to support the current Ubuntu versions that has not reached end of life.
   Also, we will not support versions that has reached end of standard support.
   
   Please see `<https://wiki.ubuntu.com/Releases>`_ for more information.

RobWork is basically multiple projects:

- RobWork :
  is the core part including math, kinematics, planning and so on.
- RobWorkStudio :
  is the GUI which enable visualization and more user friendly interfaces through gui plugins
- RobWorkSim :
  is an extension to the RobWork core functionality which adds dynamic simulation of bodies,
  devices and several tactile sensors.

Note that RobWork is needed to run RobWorkStudio and RobWorkSim.
Therefore it is not possible to use these, without
having RobWork installed on the machine.
For new users, RobWork and RobWorkStudio will usually be sufficient.

Installing dependencies
=======================

RobWork depends on third-party software that must be installed prior to
compilation. This includes both build tools and third-party libraries.
In Linux it is quite easy to set up the dependencies as these are
available as packages in the systems package manager.

The commands shown in the following must be run from a terminal.
A terminal can usually be opened by pressing Ctrl+Alt+T in Ubuntu.

Build Tools
-----------

To be able to checkout code it is necessary to install some source code
management (SCM) tools.
To be able to checkout the code from our own Git repository, a Git client is
needed. A Git client is also required if you need to compile ODE for RobWorkSim.

To compile the C++ code, the GCC compiler should be used on Ubuntu.
CMake must be used to prepare RobWork for compilation.

.. code-block:: shell

    sudo apt-get install gcc g++ cmake

RobWork Required Dependencies
-----------------------------

RobWork has several dependencies, which can be installed with:

.. code-block:: shell

    sudo apt-get install libboost-dev \
                         libboost-filesystem-dev \
                         libboost-program-options-dev \
                         libboost-serialization-dev \
                         libboost-thread-dev \
                         libeigen3-dev \
                         libqhull-dev


RobWork Optional Dependencies
-----------------------------

Xerces can be used some places in RobWork for opening XML files. It is
no longer a strict requirement, as RobWork is now able to use a Boost
parser instead. If you enable compilation of non-standard parts of
RobWork, or need to compile old RobWork-dependent projects, it might be
a good idea to compile Xerces:

.. code-block:: shell

    sudo apt-get install libxerces-c3.2 libxerces-c-dev

For Ubuntu version 16.04, use libxerces-c3.1 instead.

SWIG (optional) is a tool that makes it possible to generate a LUA
script interface for RobWork. SWIG must be version 3 or newer (since we use C++11).
Python and Java interfaces are also possible, but require that Python or Java
SDK is installed as well. All of these interfaces can be generated if
you install the following packages:

.. code-block:: shell

    sudo apt-get install swig liblua5.3-dev python3-dev python3-numpy default-jdk

In Ubuntu 19.10 and newer you can use liblua5.4-dev instead of liblua5.3-dev.

Google Test (optional) is used for unit tests in RobWork. If you are a
developer and wants to develop code for the RobWork trunk, writing a
GTest will be a requirement:

.. code-block:: shell

    sudo apt-get install libgtest-dev

FCL (optional) is used as a collision library with a less restrictive License
then PQP, which is the current default strategy in robwork

.. code-block:: shell

    sudo apt-get install libfcl-dev

Assimp (optional) this library is used to load several 3D files or speed up loading
some natively supported 3D files in RobWork

.. code-block:: shell

    sudo apt-get install libassimp-dev

Freeglut (optional) this library is used to make graphics, in our opengl implementation

.. code-block:: shell

    sudo apt-get install libfreeglut-dev


RobWorkStudio Dependencies
--------------------------

RobWorkStudio requires Qt to be installed. Only Qt5 is supported:

.. code-block:: shell

    sudo apt-get install qtbase5-dev

RobWorkSim Dependencies
-----------------------

If you need to do dynamic simulations, you will probably need the
RobWorkSim package. If you are in doubt and just need RobWorkStudio, you
can likely skip this.

Open Dynamics Engine (ODE) can be installed through the package manager:

.. code-block:: shell

    sudo apt-get install libode-dev

Ubuntu 16.04 comes with ODE 0.13.1 (libode4), Ubuntu 18.04 with ODE 0.14 (libode6)
and version 20.04 and newer with ODE 0.16 (libode8).

Notice that the version from the package manager can sometimes be a bit
outdated. If you want the latest version, Open Dynamics Engine (ODE)
must be compiled from source. Use Git to download the source from
bitbucket (use a dot '.' as the third argument to checkout directly to
the current folder):

.. code-block:: shell

    git clone https://bitbucket.org/odedevs/ode

Make sure that the required build tools are installed:

.. code-block:: shell

    sudo apt-get install automake libtool

Open a terminal, go to the ode folder (with the bootstrap file in it)
and run:

.. code-block:: shell

    ./bootstrap
    ./configure --enable-double-precision --enable-shared --enable-ou --enable-builtin-threading-impl --disable-demos --disable-asserts
    make -j4

This will make sure that ODE is built with 4 threads with double
precision as a shared library.

Bullet Physics can also be installed through the package manager.
Ubuntu 16.04 comes with 2.83.6, Ubuntu
18.04 and 19.10 with 2.87, and Ubuntu 20.04 with 2.88. The bullet packages can be installed with the
following command:

.. code-block:: shell

    sudo apt-get install libbullet-dev libbullet-extras-dev

It is also possible to compile Bullet Physics from source, if a specific
version is needed. Clone the source code with git (use a dot '.' as the
third argument to checkout directly to the current folder):

.. code-block:: shell

    git clone https://github.com/bulletphysics/bullet3

Make a Build folder under the bullet3 folder and run CMake to configure
the build. From within the Build folder, run in a terminal:

.. code-block:: shell

    cmake -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON -DBUILD_BULLET3=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=$WORKSPACE/Release -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_C_FLAGS="-fPIC" -DBUILD_EXTRAS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF -BUILD_CPU_DEMOS=OFF ..
    make -j4

Modify the options to suit your needs. The shown options will make sure
that Bullet is built with double precision, required compile flags and
switch off building of things that are normally unnecessary when used in
RobWorkSim.

Building RobWork
================

When the dependencies have been installed, RobWork is ready to be built.
First, the source must be downloaded, followed by the build procedure.

Getting source files from Git
-----------------------------

Make a new directory where you want to install RobWork (in this guide,
we will install in ~/RobWork):

.. code-block:: shell

    mkdir RobWork
    cd RobWork

When the dependencies are installed, go ahead and download the newest
version of RobWork from the Git repository at:

https://gitlab.com/sdurobotics/RobWork

In the terminal, this is done as follows: (be sure that you are located
in the directory where you want to install RobWork)

.. code-block:: shell

    git clone https://gitlab.com/sdurobotics/RobWork.git .

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
be set up one time for all in the users home folder in the .bashrc file,
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

Now we are ready to build RobWork. Run CMake in the newly created build
directory for RobWork, and run make afterwards to build the project:

.. code-block:: shell

    cd ~/RobWork/Build/RW
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWork
    make -j4
    # to build the python lua anf java language interfaces you must have swig installed and then call
    make -j4 python
    make -j4 lua
    make -j4 java

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
    # to build the python lua anf java language interfaces you must have swig installed and then call
    make -j4 python
    make -j4 lua
    make -j4 java

For RobWorkSim:

.. code-block:: shell

    cd ~/RobWork/Build/RWSim
    cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkSim
    make -j4
    # to build the python lua anf java language interfaces you must have swig installed and then call
    make -j4 python
    make -j4 lua
    make -j4 java

Finally, we need to add the following paths to ~/.bashrc:

.. code:: shell

    #ROBWORK#
    export RW_ROOT=~/RobWork/RobWork/
    export RWS_ROOT=~/RobWork/RobWorkStudio/
    export RWSIM_ROOT=~/RobWork/RobWorkSim/

Remember to only add paths to the components you have actually
installed. Ie. if you only installed RobWork and RobWorkStudio, the
path for RobWorkSim should not be set.

By setting these environment variables, it will be possible for other
projects to find the RobWork projects.
