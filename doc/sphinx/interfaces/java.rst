.. _interfaces_java:

Java Interface
================

The java interface is automatically enabled when running cmake if a Java SDK with Java Native Interface (JNI)
is found. For details on SWIG for Java, see the `SWIG 4.0 Java documentation <http://www.swig.org/Doc4.0/SWIGDocumentation.html#Java>`_
which also contains a good description of the more advanced issues when using JNI to call C++ code from Java.

Look for the line "RobWork: Java bindings ENABLED!" in the CMake output to be sure that Java interfaces are enabled.
The interface is generated for both RobWork, RobWorkStudio and RobWorkSim, and results in a series of files.

Linux example (for a Debug build)::
   
   RobWork/libs/debug/sdurw_jni.so 
   RobWork/libs/debug/sdurw_java.jar 
   RobWorkStudio/libs/debug/sdurws_jni.so 
   RobWorkStudio/libs/debug/sdurws_java.jar 
   RobWorkSim/libs/debug/sdurwsim_jni.so 
   RobWorkSim/libs/debug/sdurwsim_java.jar 


Furthermore Javadoc is generated and can be launched from::

   RobWork/libs/debug/javadoc/sdurw/index.html 
   RobWorkStudio/libs/debug/javadoc/index.html 
   RobWorkSim/libs/debug/javadoc/index.html 

SWIG generates Javadoc from SWIG 4.0.
In the future, we will add Javadoc to the script interface gradually.
Until then, please consult the ordinary :ref:`api_cpp` for detailed documentation about classes and methods.

Compiling a Java program
------------------------

To compile a basic program that can utilize the RobWork Java API, consider the following small example.

Main.java

.. code-block:: java

   import org.robwork.*;
   public class Main {
       public static void main(String[] args){
           LoaderRW.load("sdurw");
           LoaderRWS.load();
           LoaderRWSim.load();
       }
   }

To compile this piece of Java code the classpath must be set up such that the jar files can be found.
The compile command should be similar to:

.. code-block:: bash

   javac
       -classpath .:/path/to/RobWork/libs/debug/sdurw_java.jar:/path/to/RobWorkStudio/libs/debug/sdurws_java.jar:/path/to/RobWorkSim/libs/debug/sdurwsim_java.jar
       Main.java


In the Eclipse IDE it is enough to add the jar files to the build path for the Java project.
Similar can be done for other IDEs.

Note that all generated Java classes will be located in various packages under org.robwork, according to the module they belong to.
Before calling any other method on the interface, it is important that the loader functions has been called first
(always try to call these three lines as the first thing in your program).
The *LoaderRW* function takes a string which specifies which of the modules from RobWork to load (RobWorkStudio and RobWorkSim currently has only one each).

Now to actually run the program use the same classpath as before, and set the java.library.path:

.. code-block:: bash

   java
       -classpath .:/path/to/RobWork/libs/debug/sdurw_java.jar:/path/to/RobWorkStudio/libs/debug/sdurws_java.jar:/path/to/RobWorkSim/libs/debug/sdurwsim_java.jar
       -Djava.library.path=/path/to/RobWork/libs/debug:/path/to/RobWorkStudio/libs/debug:/path/to/RobWorkSim/libs/debug
       Main


Note that you need to use the -Djava.library.path option to set the library path. This is the path where
Java will search for the JNI .so (Linux) or .dll (Windows) files. In the Eclipse IDE this would be set in
the Run Configuration under arguments to the Virtual Machine.

There is an alternative to set the -Djava.library.path. If this is not specified at runtime, it can be hard-coded.
This is done by explicitly defining the path where the .so or .dll files are located when running the load methods.

.. code-block:: java

       LoaderRW.load("/path/to/RobWork/libs/debug", "sdurw");
       LoaderRWS.load("/path/to/RobWorkStudio/libs/debug");
       LoaderRWSim.load("/path/to/RobWorkSim/libs/debug");


Examples
--------

To see examples of how the RobWork interface is used in Java, please look in the examples folder
for the different projects. For example look in the folders::

   RobWork/example/java
   RobWorkStudio/example/java
   RobWorkSim/example/java


Naming Conventions
------------------

Java has no concept of operator overloading which is used extensively in the C++ API.
To solve the problem of operator overloading in Java, the following naming conventions are used:

+------------------+---------------------+
| C++              | Java                |
+==================+=====================+
| operator-()      | negate()            |
+------------------+---------------------+
| operator*()      | multiply()          |
+------------------+---------------------+
| operator/()      | divide()            |
+------------------+---------------------+
| operator==()     | equals()            |
+------------------+---------------------+

Memory, Pointers, Arrays & References
-------------------------------------

In C++ there is a distinction between pass and return by reference, pointer or value.
This is not the case for Java. The Java object proxy is technically always the equivalent of a C++ pointer.

The Java objects can own the corresponding C++ object in the native interface. If it owns the native object
is will call the C++ destructor once the Java object is Garbage Collected. The C++ destructor can also be
called explicitly with the delete() function. In this case the Java object will be invalid, and it is up
to the user not to call methods on a object where delete has been called. If a object is returned from a C++ function
by value, the equivalent method in Java will return a Java object that owns the underlying C++ object. Similar
goes for objects constructed in Java by use of the new keyword.

The Java object does not need to be the owner of the C++ object. If a object is returned as a
reference or pointer in C++, the equivalent method in Java will return the same Java type object as before, but
this time it will not be the owner of the underlying C++ object. This distinction between return by value and
return by reference/pointer is in many ways what one would expect from standard C++ behaviour.

For input arguments to methods called in Java, every Java object passed can be considered a pointer. If the C++ function takes a value the object will
be copied, and if the C++ function takes a reference or pointer it will be passed by reference while being owned
by Java.

Now consider the following small example of creating a smart pointer:

.. code-block:: java

   PathTimedStatePtr path = ownedPtr(new PathTimedState());

When the new PathTimedState() constructor is called an owned object is constructed.
Note however that this is done anonymously and that there is no reference to the newly created object.
Clearly this object might be deleted by the Garbage Collector right after creation. As it is owned
it will also destruct the underlying C++ object.

The *ownedPtr* has been implemented such that it takes ownership of the
object it is created from. In this example the *ownedPtr* changes the ownership of the anonymous
PathTimedState object to false. This way only the Java object will be Garbage Collected, but the C++
object will remain. The smart pointer will make sure that there is still a way to access the object.
Note that the PathTimedStatePtr object is always owned no matter what. This is important as the native
smart pointer object must follow the lifetime of the smart pointer in Java in order to maintain the correct
reference count.

Note that using the above code snippet is always fine as long as Ptr types are constructed. Care must in general
be taken when anonymous objects are created, or owned objects might go out of scope. Garbage Collector issues
might be difficult to debug as it is unknown when the Garbage Collector might run, and might cause weird issues in
the program.

A good advice is to always prefer the smart pointer objects. They will always keep the C++ objects alive while
there still exists references to it (either in native code or in Java). In general one does not need to call the
C++ destructors explicitly with the delete function. If delete is called, consider setting the object to null right
after to avoid calling unavailable methods (these errors might be hard to debug).

Callbacks
---------------------------------------------

Callbacks are typically required when doing simulations. Java does not support function pointers in the same sense
as C++ does, so instead the callback can either be implemented by implementing a callback interface, or by
creating a listener for callback events.

First consider the callback interface for the ThreadSimulator:

.. code-block:: java

   package org.robwork;

   public interface ThreadSimulatorStepCallbackHandler {
       public void callback(ThreadSimulator simulator, State state);
   }

To register a callback on the simulator a implementation of the interface must be provided by the user.

The native ThreadSimulator expects a boost function object in the setStepCallBack function.
Looking at the ThreadSimulator Java API, there are two available setStepCallBack methods that takes on of the
following types as input:


* ThreadSimulatorStepCallback
  Represents a boost function and is equivalent to the native StepCallback typedef in ThreadSimulator.
  It is not possible to create this type from Java, but if predefined callback implementations are provided
  in native code this will be the type to use.

* ThreadSimulatorStepCallbackEnv
  Represents an extended boost function. To make callbacks to Java, information about the Java environment must be stored.
  The extended type allows this.
  All callbacks defined in Java is created from this type, and this type is not compatible with the ThreadSimulatorStepCallback type.

The following three lines of code sets the callback method on the ThreadSimulator (tsim): 

.. code-block:: java

   ThreadSimulatorStepCallbackHandler cb = new Callback();
   ThreadSimulatorStepCallbackEnv fct = new ThreadSimulatorStepCallbackEnv(cb);
   tsim.setStepCallBack(fct);

where Callback is the implementation of the ThreadSimulatorStepCallbackHandler interface.

Now consider the memory mangement aspect. First consider the ThreadSimulatorStepCallbackHandler object.
When creating the ThreadSimulatorStepCallbackEnv object, the native code keeps a reference to the
ThreadSimulatorStepCallbackHandle object (as it need to call this back asynchronously).
Hence the JVM will not do Garbage Collection on this object, and we are safe.

Next consider the ThreadSimulatorStepCallbackEnv object. Clearly this object is a candidate for Garbage Collection.
Note however that the native C++ function setStepCallBack specifies that arguments are passed by value. Therefore
the ThreadSimulatorStepCallbackEnv object is copied, and Garbage Collection can safely delete the object afterwards.
As the ThreadSimulatorStepCallbackEnv object is owned, the underlying C++ object is also removed. This if fine as it
has already been copied.

A second approach for creating callbacks, is by implementing event handlers. This is mainly implemented
for use in MATLAB applications, but might make sense in Java applications as well. Basically this method
wraps the first method mentioned.

Again the callback is implemented by implementing the following interface. The simulator and state is
stored inside a ThredSimulatorStepEvent.

.. code-block:: java

   package org.robwork;

   public interface ThreadSimulatorStepEventListener extends EventListener {
       void stepEvent(ThreadSimulatorStepEvent event);
   }

Next the event listener can be added to the simulator with the following few lines of code:

.. code-block:: java

   ThreadSimulatorStepEventDispatcher dispatcher = new ThreadSimulatorStepEventDispatcher();
   ThreadSimulatorStepEventListener listener = new Listener();
   dispatcher.addThreadSimulatorStepEventListener(listener);
   ThreadSimulatorStepCallbackEnv fct = new ThreadSimulatorStepCallbackEnv(dispatcher);
   tsim.setStepCallBack(fct);

where Listener is the implementation of ThreadSimulatorStepEventListener.
