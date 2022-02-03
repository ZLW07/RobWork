Bullet
******

Bullet Physics must be compiled from source. Clone the source code
with git from the source: https://github.com/bulletphysics/bullet3

Bullet takes up around 440 MB, and takes around 15 minutes to compile.

Bullet cna be compiled with the following commands, but first a few notes:
- change Line one the where you want to install bullet
- change Line four to the version you want
- Choose the generator that fits your Visual Studio version with the -G
option. Modify the options to suit your needs. The shown options will make sure that Bullet is built
with double precision, shared runtime and switch off building of things
that are normally unnecessary when used in RobWorkSim. Notice that
switching off USE\_MSVC\_DISABLE\_RTTI is only required from Bullet 2.87
and newer. 

::
    set Install_DIR=C:\Local
    git clone https://github.com/bulletphysics/bullet3
    cd bullet3
    git checkout 3.05
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64  -DUSE_DOUBLE_PRECISION=ON -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DUSE_MSVC_DISABLE_RTTI=OFF -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF -DINSTALL_LIBS=ON -DCMAKE_INSTALL_PREFIX=%Install_DIR%\Bullet
    msbuild BULLET_PHYSICS.sln -m /property:Configuration=Release
    mkdir %Install_DIR%\Bullet
    msbuild INSTALL.vcxproj -m /property:Configuration=Release
