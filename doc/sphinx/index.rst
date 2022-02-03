.. image:: https://robwork.dk/getBadge
   :target: https://gitlab.com/sdurobotics/RobWork

--------------

Introduction to RobWork
===================================

.. note::
   We are currently working a new documentation for RobWork based on Sphinx (this page).
   The old doxygen based documentation can still be found `here <apidoc/cpp/doxygen/index.html>`_ .
   Doxygen will continue to be used as a reference for the C++ API, while overall documentation and tutorials will be migrated.

*RobWork* is a collection of C++ libraries for simulation and control
of robot systems. RobWork is used for research and education as well
as for practical robot applications. Features of the library include:

- Kinematic modeling of various types of industrial manipulators, serial-, tree-, and parallel structures.
- General path-planning, path optimization, collision detection and inverse kinematics algorithms.
- Both kinematic and dynamic simulation of manipulators, controllers and sensor.
- Plugins for grasp simulation of suction cups, parallel- and dexterous grippers.
- Simple and extendible GUI and plugin system for integrating user algorithms and visualizations.
- A script interface based on SWIG which extends RobWork to multiple script languages such as Python, Lua & Java.

Besides the core part, RobWork has a number of add-ons including

- RobWorkStudio which provides a graphical user interface.
- RobWorkSim which is for dynamic simulations.
- RobWorkHardware which is a collection of hardware drivers that connect into RobWork.

Target audience of RobWork is:

- Researchers who needs a common framework for experimental robotics
- Students who wants to experiment with concepts of robotics
- Implementers of robot applications

RobWork is developed by `SDU Robotics <http://www.sdu.dk/en/Om_SDU/Institutter_centre/SDURobotics>`_ at the `University of Southern Denmark <http://www.sdu.dk/en>`_ .
The project was started in 2006 by Ph.D. and master students students at the `Maersk Mc-Kinney Moeller Institute <https://www.sdu.dk/en/om_sdu/institutter_centre/mmmi_maersk_mckinney_moeller>`_.
The focus is on is on cognitive and applied robotics.
This includes fields such as industrial robots, robot control, mathematical modelling and simulation, welfare robotics, and surgical robotics.

Introduction to the documentation
=====================================

This documentation page includes tutorials on how to compile, install and use RobWork:

- :ref:`installation` guides you through setting up RobWork on Ubuntu, CentOS and Windows.
- :ref:`manual` provides tutorials on basic usage of RobWork and RobWorkStudio. New users should start with these tutorials.
- :ref:`tutorials` provides tutorials on more advanced topics.
- :ref:`file_formats` is a reference for the XML format used to describe scenes in RobWork.
- :ref:`interfaces` gives details about developing in different languages, such as C++, Python, Java & Lua.
- :ref:`hardware` has an overview of the hardware that RobWork provides drivers for.
- :ref:`contributing` gives information about coding standards and the procedure for contributing code to the RobWork project.
- :ref:`scene_collection` shows the device and scene models available in RobWorkData.
- :ref:`apidoc` provides a reference for the four supported languages: C++, Python, Java & Lua.
- :ref:`developers` is an overview of the main developers of RobWork.
- :ref:`publications` gives an overview on publications that are based on RobWork.
- :ref:`license` gives an overview of licenses used by RobWork and all of RobWorks dependencies.

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Contents:

   installation
   manual
   tutorials
   file_formats
   interfaces
   hardware
   contributing
   scene_collection
   apidoc/index
   developers
   publications
   license

.. Indices and tables
   ==================

   * :ref:`genindex`
   * :ref:`modindex`
   * :ref:`search`
