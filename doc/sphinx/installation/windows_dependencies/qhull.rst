QHull
*****

QHull can be compiled an installed with the following script

::
    set Install_DIR=C:\Local
    git clone https://github.com/qhull/qhull.git
    cd qhull
    git checkout v8.0.2
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64 -DCMAKE_INSTALL_PREFIX=%Install_DIR%\qhull -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
    msbuild qhull.sln -m /property:Configuration=Release
    mkdir %Install_DIR%\qhull
    msbuild INSTALL.vcxproj -m /property:Configuration=Release
    