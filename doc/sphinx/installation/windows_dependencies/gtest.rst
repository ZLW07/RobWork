Google Test
***********

(optional) is used for unit tests in RobWork. If you are
a developer and wants to develop code for the RobWork trunk, writing a
GTest will be a requirement.

Go to the folder where you want to put the Google Test source.
Right-click and click "Git GUI Here". Now insert
https://github.com/google/googletest.git as the source location, and
choose the target directory. The target directory must be an empty or
non-existing directory. Finally, press clone to clone the Git
repository.

.. figure:: ../../gfx/installation/GTest_clone.png

    Cloning Google Test source with the Git GUI.

After cloning, you should see the following directory layout:

.. figure:: ../../gfx/installation/GTest_layout.png

    Google Test directory layout.

Note down the path to the googletest folder. We will refer to this as
GTEST\_ROOT and GTEST\_SOURCE later on.

The Google Test code should not be compiled. It will be compiled as a
part of the RobWork compilation when the source code is present. The
Google test repository uses up to 95 MB.
