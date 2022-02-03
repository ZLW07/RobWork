Xerces
******
(optional) can be used some places in RobWork for opening XML
files. It is no longer a strict requirement, as RobWork is now able to
use a Boost parser instead. If you enable compilation of non-standard
parts of RobWork, or need to compile old RobWork-dependent projects, it
might be a good idea to compile Xerces.

Go to http://xerces.apache.org (older versions can be found here:
http://archive.apache.org/dist/xerces/c/3/sources) and download and
unpack the source distribution.

Xerces 3.2 and newer are CMake based, and you can use the new procedure
to compile it:

First, go to the unpacked Xerces folder and create two folders inside
it, called build and xerces-install:

.. figure:: ../../gfx/installation/Xerces_createbuildfolder.png

    The Xerces source. Create empty folder build and xerces-install manually.

Open a Visual Studio "x64 Native Tools Command Prompt", and go to the
newly created build folder. Now run the following command:

::

    cmake -G "Visual Studio 15 2017 Win64" -DCMAKE_INSTALL_PREFIX:PATH="C:/some/path/to/xerces-install"

If CMake succeeds, go to the build folder, and open xerces-c.sln. Then
chosse Release mode and 64 bit build as follows:

.. figure:: ../../gfx/installation/xerces_install_1.png

    Choose the 'Release' configuration (alternatively choose 'Static Release' if you prefer static libraries).

.. figure:: ../../gfx/installation/xerces_install_2.png

    Choose 64 bit build.

Build the XercesLib target:

.. figure:: ../../gfx/installation/xerces_install_3.png

    Right click XercesLib in the SolutionExplorer and click 'Build'.

Finally run build for the INSTALL target. This will populate the
xerces-install folder with a bin, cmake, include, lib and share folder.
Note down the path to the xerces-install folder. We will use the name
XERCESC\_ROOT to refer to that directory path later when setting up the
RobWork project.

Xerces will take up around 250 MB in total, and will take around 20
minutes to download and compile.

Old installation procedure (Xerces 3.1.4 and earlier):

- Go to http://xerces.apache.org (older versions can be found here: http://archive.apache.org/dist/xerces/c/3/sources) and download the source distribution.
- Unpack it where you want Xerces installed.
- Open xerces-c-3.1.4/projects/Win32/VCxx/xerces-all.sln in Visual Studio (substitute VCxx with your Visual Studio version - see https://en.wikipedia.org/wiki/Microsoft\_Visual\_Studio#History for overview).
- Choose 64-bit Release build configuration, and build the XercesLib target.

Compile Xerces
--------------

::
    set Install_DIR=C:\Local
    svn checkout https://svn.apache.org/repos/asf/xerces/c/tags/Xerces-C_3_2_2/
    cd Xerces-C_3_2_2
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64 -DBUILD_SHARED_LIBS:BOOL=OFF -DCMAKE_INSTALL_PREFIX=%Install_DIR%\XercesC
    msbuild xerces-c.sln -m /property:Configuration=Release
    mkdir %Install_DIR%\XercesC
    cmake --build . --config Release --target install