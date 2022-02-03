Open Dynamics Engine (ODE)
**************************

If you need to do dynamic simulations, you will probably need the
RobWorkSim package. If you are in doubt and just need RobWorkStudio, you
can likely skip this section.

**Open Dynamics Engine (ODE)** must be compiled from source. Use
**Git** to download the source from bitbucket at the address:

https://bitbucket.org/odedevs/ode

CMake is used by ODE 0.15.2 and newer. It takes 10 minutes to setup and
compile, and takes up around 85 MB. This is the recommended procedure:

::
    set Install_DIR=C:\Local
    git clone https://bitbucket.org/odedevs/ode
    cd ode
    git checkout 0.16.2
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=%Install_DIR%\ode -DBUILD_SHARED_LIBS:BOOL=OFF -DODE_WITH_DEMOS=OFF -DODE_WITH_OU=ON -DODE_WITH_TESTS=OFF -DODE_DOULBE_PRECISION=ON
    msbuild ode.sln /property:Configuration=Release
    mkdir %Install_DIR%\ode
    msbuild INSTALL.vcxproj /property:Configuration=Release
