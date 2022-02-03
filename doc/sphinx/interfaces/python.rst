.. _interfaces_python:

Python interface
================

Make sure you have both SWIG and python installed. If you are using Ubuntu, make sure to install python3-dev package as well.
You can check if Python interfaces for RobWork are generated when running cmake. There should be lines like: "RobWork: python bindings ENABLED" and
"RobWorkStudio: python bindings ENABLED".

When compiling RobWork, RobWorkStudio and RobWorkSim the python interfaces will be
generated. These consist of two files per project (RobWork has additional for its extra libraries) and they are placed in the
libs folder under the relevant build configuration.

Linux example for Debug build configuration::
   RobWork/Build/RW/libs/Debug/sdurw/_sdurw.so
   RobWork/Build/RW/libs/Debug/sdurw/sdurw.py
   RobWork/Build/RW/libs/Debug/sdurw/__init__.py
   RobWork/Build/RWStudio/libs/Debug/sdurws/_sdurws.so
   RobWork/Build/RWStudio/libs/Debug/sdurws/sdurws.py
   RobWork/Build/RWStudio/libs/Debug/sdurws/__init__.py
   RobWork/Build/RWSim/libs/Debug/sdurwsim/_sdurwsim.so
   RobWork/Build/RWSim/libs/Debug/sdurwsim/sdurwsim.py
   RobWork/Build/RWSim/libs/Debug/sdurwsim/__init__.py


For the above example loading of the modules in the python interpreter requires the PYTHONPATH environment variable to be set:

.. code-block:: bash

   export PYTHONPATH=/path/to/RobWork/Build/RW/libs/BUILD_CONFIGURATION/:/path/to/RobWork/Build/RWStudio/libs/BUILD_CONFIGURATION/:/path/to/RobWork/Build/RWSim/libs/BUILD_CONFIGURATION/:$PYTHONPATH

As an alternative the paths can be set in the python script as well:

.. code-block:: py

   import sys
   sys.path.append('/path/to/RobWork/Build/RW/libs/BUILD_CONFIGURATION')
   sys.path.append('/path/to/RobWork/Build/RWSim/libs/BUILD_CONFIGURATION')
   sys.path.append('/path/to/RobWork/Build/RWStudio/libs/BUILD_CONFIGURATION')
   import sdurw, sdurws, sdurwsim
   // now we can use all robwork python bindings

Now all RobWork types, that have bindings, should be available.

Coding in Python
----------------
When coding in Python it is very important to remember that the underlying objects are still c++,
this can most often be seen when a segmentation fault occurs. Here is a very python thing to do,
that is bad practice when working with a cpp object

.. code-block:: py

   a = someRWFunction() # we get an object from a function
   print(a.b)           # the object has an inner object that we are interested in
   a = a.b              # since we don't need [a] we override it with [b]

   #CPP is now informed that python no longer has a reference to [a] and deletes the object.
   #Since [b] belongs to [a] b is deleted in the process, python therefore has a reference to free memory
   #Since the memory isn't instantaneously overridden a few things can happen

   b.getName()          # will most likely work
   res =someFunction(b) # will most likely return with bad result

   #generate a lot of new data
   c=[]
   for i in range (10000):
      c = sdurw.Vector3d()

   #From this point on segfaults might occur
   b.getName()
   someFunction(b)

Other examples might be taking items out of a std::vector and then discarding the vector once the useful
objects has been found, but because the vector is maintaining the objects in c++ once the Vector is gone
unless you copied the data instead of referencing it you'll have the same problem as above.

Running RobWorkStudio
---------------------

For some purposes it might be useful to start an instance of RobWorkStudio:

.. code-block:: py

   rwstudio = sdurws.getRobWorkStudioInstance()
   # now load a workcell
   rwstudio.setWorkCell(sdurw.WorkCellLoaderFactory.load('some/workcell.wc.xml'))
   # lets get the workcell
   wc = rwstudio.getWorkCell()
   print(wc.getName())

Communicating with plugins
--------------------------

It is often necessary to send messages or data to one or more plugins. For this the
generic event methods on RobWorkStudio is used. These are wrapped in utils such that
the current send methods can be used in python

.. code-block:: py

   rwstudio.send("someStr")
   rwstudio.send("msgId", "some string")
   rwstudio.send("msgId", 0.45235)
   rwstudio.send("msgId", sdurw.Q(4, 0.1, 0.2, 0.3, 0.5) )
   rwstudio.send("msgId", somePropertyMap )

The first send method use RobWorkStudio::genericEvent the next 4 use RobWorkStudio::genericAnyEvent.
Please take a look in rws/RobWorkStudio.hpp to get an example on using these events in your plugin.

Using path planners in python
-----------------------------

RobWork has several path planners which might be used from python. If we assume that a workcell with a
6 DOF robot named **UR1** has been loaded then a planner can be executed as follows:

.. code-block:: py

   # we need the workcell to get a handle to the robot
   wc = rwstudio.getWorkCell()
   dev = wc.findDevice("UR1")
   state = rwstudio.getState()
   cd = rwstudio.getCollisionDetector()
   planner = sdurw.QToQPlanner_makeRRT(cd,dev,state)

   # now the planner is ready to be used. We define the configurations
   # in which the robot should start and end
   q_from = sdurw.Q(6,0,-1,0,0,0,0)
   q_to = sdurw.Q(6, 3,0.2,1,-1,0,0)
   result = planner.query(q_from,q_to)

We could also chose a query with a timeout or with some other rw::pathplanning::StopCriteria

.. code-block:: py

   result = planner.query(q_from,q_to, 10.0)
   stopCriteria = sdurw.StopCriteria_stopCnt(100)
   result = planner.query(q_from,q_to,stopCriteria)
