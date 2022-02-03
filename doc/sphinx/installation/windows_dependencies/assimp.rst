Assimp
******

Assimp can be compiled on windows by running the following script.

::
    set Install_DIR=C:\Local
    git clone https://github.com/assimp/assimp.git
    cd assimp
    git checkout v5.0.1
    mkdir Build
    cd Build
    cmake .. -G "Visual Studio 16 2019" -A x64  -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DASSIMP_BUILD_SAMPLES=OFF -DASSIMP_BUILD_TESTS=OFF -DASSIMP_LIB_INSTALL_DIR:PATH=%Install_DIR%\Assimp\lib -DASSIMP_INCLUDE_INSTALL_DIR:PATH=%Install_DIR%\Assimp\include -DASSIMP_BIN_INSTALL_DIR:PATH=%Install_DIR%\Assimp\bin
    msbuild Assimp.sln -m /property:Configuration=Release
    mkdir %Install_DIR%\Assimp
    msbuild INSTALL.vcxproj /property:Configuration=Release