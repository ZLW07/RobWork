Eigen3
******

Eigen can be installed on windows by running the following script.

::
    set Install_DIR=C:\Local
    git clone https://gitlab.com/libeigen/eigen.git
    cd eigen
    git checkout 3.3.7
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64  -DCMAKE_INSTALL_PREFIX=%Install_DIR%\eigen
    mkdir %Install_DIR%\eigen
    msbuild INSTALL.vcxproj /property:Configuration=Release
