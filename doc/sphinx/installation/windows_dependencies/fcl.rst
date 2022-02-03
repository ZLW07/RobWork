fcl
***

Fcl can be compiled and installed with the following script

::
    set Install_DIR=C:\Local
    git clone https://github.com/flexible-collision-library/fcl.git
    cd fcl
    git checkout 0.6.1
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64  -DCMAKE_INSTALL_PREFIX=%Install_DIR%/fcl -DEIGEN3_INCLUDE_DIR=%Install_DIR%/eigen/include/eigen3 -DCMAKE_PREFIX_PATH:PATH=%Install_DIR%/libccd -DCMAKE_INCLUDE_PATH:PATH=%Install_DIR%/libccd/lib -DFCL_BUILD_TESTS=OFF -DFCL_STATIC_LIBRARY=ON
    msbuild FCL.sln -m /property:Configuration=Release
    mkdir %Install_DIR%\fcl
    msbuild INSTALL.vcxproj -m /property:Configuration=Release