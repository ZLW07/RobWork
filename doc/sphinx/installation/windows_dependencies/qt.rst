Qt
--

RobWorkStudio requires to be installed. Only Qt5 is
supported. It is encouraged to use at least Qt 5.9. Download and install Qt from:

https://www.qt.io

You need to choose the Open Source version. Notice that Qt is only free
for open source projects. Also, you need to register to download Qt.

WARNING! Please avoid Qt 5.8 ( see issue
https://gitlab.com/sdurobotics/RobWork/issues/37 )

Run the Online installer for Windows, and select the components you
want. Simply select your Visual Studio version under the version of Qt
you want to use.

.. figure:: ../../gfx/installation/Qt5_components.png

    Choice of Qt components. It is enough to make a single selection with your Visual Studio version.

Qt installer might launch QtCreator at the end. Just close this program,
as we intend to use Visual Studio instead. Qt5 will use aroung 3.65 GB
disk space.

After installation you should have a folder with the following layout:

.. figure:: ../../gfx/installation/Qt5_layout.png

    Qt5 directory layout.

Note down the path to the Qt folder shown above, we will need that when
setting up the RobWorkStudio project.