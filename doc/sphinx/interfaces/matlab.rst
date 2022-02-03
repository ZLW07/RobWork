MatLab Interface
================

As MATLAB has good Java support, it is possible to interface RobWork from MATLAB via Java.
In general it is recommended that a basic Java program is first compiled and tested before
trying to use the Java interface from MATLAB. This is due to the fact that there might be conflicts
between the libraries bundled with MATLAB, and the dependencies of the native JNI libraries.

Launching MATLAB
----------------

When MATLAB is to be used together with the RobWork JNI libraries, MATLAB should be launched with some
environment variables set in order to control the loading of dependent libraries.

The following table gives an overview of some of the version numbers for different MATLAB versions (Linux):

.. list-table::
   :header-rows: 1

   * - Version
     - Name
     - JVM
     - GCC/G++
     - Boost
     - Best Ubuntu Match
   * - MATLAB 7.2
     - R2006a
     - 1.5.0
     - ?
     - ?
     - -
   * - MATLAB 7.3
     - R2006b
     - 1.5.0
     - 3.4
     - ?
     - -
   * - MATLAB 7.4
     - R2007a
     - 1.6.0
     - 4.1
     - ?
     - -
   * - MATLAB 7.5
     - R2007b
     - 1.6.0
     - 4.1
     - ?
     - -
   * - MATLAB 7.6
     - R2008a
     - 1.6.0_04
     - 4.1
     - ?
     - -
   * - MATLAB 7.7
     - R2008b
     - 1.6.0_04
     - 4.1
     - ?
     - -
   * - MATLAB 7.8
     - R2009a
     - 1.6.0_12
     - 4.2
     - ?
     - -
   * - MATLAB 7.9
     - R2009b
     - 1.6.0_12
     - 4.2
     - ?
     - -
   * - MATLAB 7.10
     - R2010a
     - 1.6.0_12
     - 4.2
     - ?
     - -
   * - MATLAB 7.11
     - R2010b
     - 1.6.0_17
     - 4.3
     - ?
     - -
   * - MATLAB 7.12
     - R2011a
     - 1.6.0_17
     - 4.3
     - 1.40.0
     - 10.04, 10.10
   * - MATLAB 7.13
     - R2011b
     - 1.6.0_17
     - 4.3
     - 1.44.0
     - -
   * - MATLAB 7.14
     - R2012a
     - 1.6.0_17
     - 4.4
     - 1.44.0
     - -
   * - MATLAB 8
     - R2012b
     - 1.6.0_17
     - 4.4
     - 1.44.0
     - -
   * - MATLAB 8.1
     - R2013a
     - 1.6.0_17
     - 4.42
     - 1.49.0
     - 12.10, 13.04
   * - MATLAB 8.2
     - R2013b
     - 1.7.0_11
     - ?
     - ?
     - -


The following approach has been tested with MATLAB R2013a on a Ubuntu 13.04 system. It is expected that
the library resolution issues might be very different for other versions of both OS and MATLAB.
It is also uncertain how this will be handled in a Windows environment.

Looking in the table, the Boost libraries bundled with MATLAB R2013a are version 1.49. This is lucky
as this is also the default version of the libraries in Ubuntu 13.04. Note that Boost libraries of different
version numbers can not in general be expected to have good compatibility, hence mixing them should be avoided.
It might be necessary to compile against the MATLAB Boost libraries instead. This will require downloading the
correct headers for the required version of Boost and setting up new Boost paths before running cmake.

First of all the problem with library mismatches is clearly illustrated when using ldd to resolve the
dependencies of the rw_jni.so library.

Try to execute the following in a terminal:

.. code-block:: bash

   ldd /home/user/RobWork/libs/debug/libsdurw_jni.so


Now try to run in MATLAB:

.. code-block:: matlab

   !ldd /home/user/RobWork/libs/debug/libsdurw_jni.so


Notice the difference between how the library dependencies are resolved. MATLAB comes with its own version of the
system libraries, and these will often be older than the system dependencies that the JNI library was linked
against.

It is adviced that a bash script similar to the following is used to launch MATLAB to force it to use the
system environment.

.. code-block:: sh

   #!/bin/bash
   export RW_ROOT=/home/user/RobWork
   export RW_BUILD=debug

   export RW_LIBS=$RW_ROOT/RobWork/libs/$RW_BUILD
   export RWStudio_LIBS=$RW_ROOT/RobWorkStudio/libs/$RW_BUILD
   export RWSim_LIBS=$RW_ROOT/RobWorkSim/libs/$RW_BUILD

   export WORK_DIR=$RW_ROOT/RobWorkSim/example/scripts/matlab
   cd $WORK_DIR
   echo "$RW_LIBS/sdurw_java.jar" > javaclasspath.txt
   echo "$RWStudio_LIBS/sdurws_java.jar" >> javaclasspath.txt
   echo "$RWSim_LIBS/sdurwsim_java.jar" >> javaclasspath.txt

   export MATLAB_JAVA=$JAVA_HOME/jre
   export LD_PRELOAD="$LD_PRELOAD /usr/lib/x86_64-linux-gnu/libstdc++.so.6"

   /opt/MATLAB/bin/matlab

The RW* variables are set for convenience to allow writing MATLAB scripts that are system independent.

Set the WORK_DIR varible to you project directory. A javaclasspath.txt file will by created in this directory
to set the static Java classpath in MATLAB. MATLAB also allows setting this dynamically from within MATLAB,
but please avoid this (especially if callbacks from Java to MATLAB are required).

The MATLAB_JAVA variable should be set if the .jar files has been compiled to a newer version of Java
than the JVM used by MATLAB. This will make sure that MATLAB uses the current system JVM.
This line might be uncommented if the versions already match.

The LD_PRELOAD variable forces MATLAB to use newer system libraries for libstdc++.so.6 instead of the
libraries that comes with MATLAB. This should be safe as the libstdc++ library is designed to be backwards
compatible.

Note that overriding the libraries that MATLAB use and changing the JVM is a drastic change that might
give other issues in MATLAB. Depending on the system it might not always be a requirement to set these variables.
Always try setting as few variables first and then add MATLAB_JAVA and LD_PRELOAD if required.
It might also be necessary to add even more libraries than shown here.

The following MATLAB code should run without errors before the RobWork API can be used from MATLAB.

.. code-block:: matlab

   RW_LIBS=getenv('RW_LIBS');
   RWSim_LIBS=getenv('RWSim_LIBS');
   RWStudio_LIBS=getenv('RWStudio_LIBS');

   % Import the java API
   javaaddpath(strcat(RW_LIBS,'/sdurw_java.jar'));
   javaaddpath(strcat(RWSim_LIBS,'/sdurwsim_java.jar'));
   javaaddpath(strcat(RWStudio_LIBS,'/sdurws_java.jar'));
   import dk.robwork.*;

   % Load the native libraries
   LoaderRW.load(RW_LIBS)
   LoaderRWSIM.load(RWSim_LIBS)
   LoaderRWS.load(RWStudio_LIBS)

Typical Errors
--------------

It can be difficult to get the MATLAB interface to run. The following is a list of known errors and
possible solutions.

.. code-block:: matlab

   >> LoaderRW.load(RW_LIBS)
   Java exception occurred:
   java.lang.UnsatisfiedLinkError: /home/user/RobWork/RobWork/libs/debug/librw_jni.so:
   /opt/MATLAB/bin/glnxa64/../../sys/os/glnxa64/libstdc++.so.6: version "GLIBCXX_3.4.15' not found (required by /home/user/RobWork/RobWork/libs/debug/librw_jni.so)"


This error is caused by librw_jni.so as it is dependent on a newer version of the standard C++ library than
the library provided and used by MATLAB. To solve this issue set the LD_PRELOAD:

.. code-block:: bash

   export LD_PRELOAD="$LD_PRELOAD /usr/lib/x86_64-linux-gnu/libstdc++.so.6"


If classes can not be found:

.. code-block:: matlab

   >> javaaddpath(strcat(RW_LIBS,'/sdurw_java.jar'));
   >> import dk.robwork.*;
   >> LoaderRW.load(RW_LIBS)
   Undefined variable "LoaderRW" or class "LoaderRW.load".


Make sure that the .jar file is at the given path, and that the .jar actually contains a LoaderRW class.
If this is the case, the reason for MATLAB not finding the class can be that there is a mismatch between
the .jar Java version and the MATLAB JVM.

There can be two solutions. First one is to use another JVM version in MATLAB.

.. code-block:: sh

   export MATLAB_JAVA=$JAVA_HOME/jre


Secondly the source can be manually compiled using a different Java compiler version. The generated source
is located in the build folder under src/rwlibs/swig/java_src and similar for the other packages. See the Java
version used by MATLAB with

.. code-block:: sh

   version -java


Callbacks
---------

When running simulations, callbacks might be required from C++ code to MATLAB. In MATLAB the implementation of
such callbacks can be a bit tricky. Callbacks are implemented as a listener to the corresponding Java event. 

First consider a MATALB function handling the callback event:

.. code-block:: matlab

   function StepCallBack( dispatcherObject, event, rwstudio )
       tsim=event.getThreadSimulator();
       state=event.getState();

       userdata=getappdata(dispatcherObject,'UserData');
       counter = userdata(1);

       % Set state in RobWorkStudio and print time for each 20 steps
       if mod(counter,20) == 0
           rwstudio.setState(state);
           display(num2str(tsim.getTime()));
       end

       setappdata(dispatcherObject,'UserData',[counter+1]);
   end

The function takes at least two arguments, namely the dispatcher object itself and the event.
In the example it is also illustrated how to append additional arguments in MATLAB (here used for
passing a pointer to the RobWorkStudio instance).

First two lines of the functions extracts the data stored in the event, which is a pointer to the
ThreadSimulator and the state. To illustrate a second way of storing additional data, a counter is
stored on the dispatcherObject under UserData (managed internally by MATLAB). On the last line the counter is
incremented. For each 20 callbacks, the state is updated in RobWorkStudio and the current simulated time
is displayed.

Now a callback can be added by using the following few lines of code (very similar to the Java example):

.. code-block:: matlab

   dispatcher = ThreadSimulatorStepEventDispatcher();
   setappdata(dispatcher,'UserData',[0]); % counter
   set(dispatcher,'StepEventCallback',{@StepCallBack,rwstudio});
   fct = ThreadSimulatorStepCallbackEnv(dispatcher);
   tsim.setStepCallBack(fct);

The third line set the callback with a MATLAB function reference to the StepCallBack function. Notice
how the rwstudio argument is added. It is possible to add an arbitrary number of static arguments this way.

Examples
-------------------------------------------

To see examples of how the RobWork Java interface can be used in MATLAB, please look in the examples folder
for the different projects::

   RobWorkStudio/example/scripts/matlab
   RobWorkSim/example/scripts/matlab