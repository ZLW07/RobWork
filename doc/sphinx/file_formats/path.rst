.. _path_format:

***************
CSV Path format
***************

In RobWork a path can be loaded from a .csv file using class rw::Loader::PathLoaderCSV.
The CSV Path format only supports QPaths for devices.

The CSV Path format offers two possible ways of defining a path. You can either define a timed or a non timed path.

The first 2 lines of the Path file defines the setup:

.. code-block::

    3;
    6;

First thing to notice is that all lines in the csv file must end with a semicolon

The first line defines the length of the path. If more joint configurations exists in the path they are ignored.
The second line defines the degree of freedom of all devices in the workcell.
So if you have one robot with 6 dof you write 6 and if you have two robots with 6dof yo write 12.

After the 2 lines of setup the path it self is defined one configuration per line.
If a workcell contains more than 1 device, the first device reads the first n values, second device reads next n values etc.

.. code-block::
    
    -1.5,0.0,-1.5,0.0,-1.5,0.0;
    -1.0,-1.5,0.0,-1.5,0.0,-1.5;
    -0.8,0.0,-1.5,0.0,-1.5,0.0;

if not enough values for a configuration is provided an exception will be thrown.
To make a timed path, the timestamp for each configuration is appended to the front. Given timestamp 3.0 ,6.0 and 9.0
the path is then defined as:

.. code-block:: 
    
    3.0,-1.5,0.0,-1.5,0.0,-1.5,0.0;
    6.0,-1.0,-1.5,0.0,-1.5,0.0,-1.5;
    9.0,-0.8,0.0,-1.5,0.0,-1.5,0.0;




