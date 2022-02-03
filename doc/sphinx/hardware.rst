.. _hardware:

Hardware
=========

RobWorkHardware is a sub package of RobWork. Its main function is to provide an interface/driver
layer for real hardware.

The package is structured into several modules which each can be enabled or disabled.
This structure has been chosen because of the multiple external dependencies and because one
typically only use few of the modules at a time.

The various external dependencies also mean that the license of some modules differ from the
general RobWork license. In such a case this will be explicitly noted in the module headers as well as
in the module documentation.

Currently driver wrappers/interfaces exist for cameras, CAN devices, serial port,
weiss tactile sensors, swissranger and various robots such as the SDH, PA10 and PowerCube.

.. toctree::
   :maxdepth: 2
   :caption: Robots:

   hardware/universal_robots
   hardware/motoman
   hardware/mitsubishi
   hardware/crs
   hardware/dockwelder
   hardware/fanuc
   hardware/katana

.. toctree::
   :maxdepth: 2
   :caption: Grippers:

   hardware/schunk
   hardware/robotiq

.. toctree::
   :maxdepth: 2
   :caption: Cameras:

   hardware/cameras

.. toctree::
   :maxdepth: 2
   :caption: Other components:

   hardware/ftcompensation
   hardware/powercube
   hardware/sick
   hardware/swissranger
   hardware/tactile
   hardware/trakstar