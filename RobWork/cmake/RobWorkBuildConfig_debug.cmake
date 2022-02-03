
# Version of RobWork
set(RW_BUILD_WITH_VERSION 22.2.3)
set(RW_BUILD_WITH_VERSION_MAJOR 22)
set(RW_BUILD_WITH_VERSION_MINOR 2)
set(RW_BUILD_WITH_VERSION_PATCH 3)

# this is the optional packages
set(RW_BUILD_WITH_OPENGL TRUE)
set(RW_BUILD_WITH_XERCES True)
set(RW_BUILD_WITH_PQP True)
set(RW_BUILD_WITH_FCL True)
set(RW_BUILD_WITH_YAOBI True)
set(RW_BUILD_WITH_LUA False)
set(RW_BUILD_WITH_SWIG False)
set(RW_BUILD_WITH_ASSIMP TRUE)
set(RW_BUILD_WITH_SANDBOX )
set(RW_BUILD_WITH_CALIBRATION )
set(RW_BUILD_WITH_MATHEMATICA FALSE)
set(RW_BUILD_WITH_FREEGLUT False)
set(RW_BUILD_WITH_NUMPY )

set(RW_BUILD_WITH_TESTS ON)
set(RW_BUILD_WITH_GTEST TRUE)
if(RW_BUILD_WITH_GTEST)
    set(RW_BUILD_WITH_GTEST_SHARED_LIBS OFF)
endif()

# flags and definitions
set(RW_BUILD_WITH_RW_ASSERT ON)
set(RW_BUILD_WITH_C_FLAGS " -fPIC")
set(RW_BUILD_WITH_CXX_FLAGS "-Wall -Wno-strict-aliasing -Wno-unused-function -fPIC -fopenmp")
set(RW_BUILD_WITH_DEFINITIONS )
set(RW_BUILD_WITH_LINKER_FLAGS )

set(RW_BUILD_WITH_RW_ROOT "/home/zw/CLionProjects/RobWork/RobWork")
set(RW_BUILD_WITH_BUILD_DIR "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWork")
set(RW_BUILD_WITH_SWIG_CMD "SWIG_EXECUTABLE-NOTFOUND")
set(RW_BUILD_WITH_SWIG_VERSION "")
set(RW_BUILD_WITH_LUA_INCLUDE_DIR "")
set(RW_BUILD_WITH_PQP_INCLUDE_DIR "/home/zw/CLionProjects/RobWork/RobWork/ext/rwpqp")
set(RW_BUILD_WITH_NUMPY_INCLUDE_DIR "")
set(RW_BUILD_WITH_YAOBI_INCLUDE_DIR "/home/zw/CLionProjects/RobWork/RobWork/ext/rwyaobi/include")
set(RW_BUILD_WITH_ASSIMP_INCLUDE_DIR "/usr/include")
set(RW_BUILD_WITH_XERCES_INCLUDE_DIR "/usr/include")
set(RW_BUILD_WITH_XERCES_LIB_DIR "")
set(RW_BUILD_WITH_GTEST_INCLUDE_DIRS "/usr/include")
set(RW_BUILD_WITH_BOOST_ROOT "")
set(RW_BUILD_WITH_BOOST_INCLUDE_DIR "/usr/include")
set(RW_BUILD_WITH_BOOST_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
set(RW_BUILD_WITH_BOOST_USE_STATIC_LIB OFF)
set(RW_BUILD_WITH_QHULL_ROOT "/usr/include/libqhull_r;/usr/include/.." )

set(RW_BUILD_WITH_INTERNAL_PQP ON)
set(RW_BUILD_WITH_INTERNAL_YAOBI ON)
set(RW_BUILD_WITH_INTERNAL_GTEST ON)

set(RW_BUILD_WITH_LUA_VERSION ".")

# and libraries
set(RW_BUILD_WITH_LIBRARIES_BOOST "Boost::filesystem;Boost::serialization;Boost::system;Boost::thread;Boost::program_options;Boost::date_time;Boost::headers")
set(RW_BUILD_WITH_LIBRARIES_LUA "")
set(RW_BUILD_WITH_LIBRARIES_OPENGL "/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so")
set(RW_BUILD_WITH_LIBRARIES_XERCESC "/usr/lib/x86_64-linux-gnu/libxerces-c.so")
set(RW_BUILD_WITH_LIBRARIES_GTEST RW::gtest_main;RW::gtest;)

set(RW_BUILD_WITH_LIBRARIES "RW::yaobi;pqp;sdurw_csgjs;sdurw_algorithms;sdurw_pathplanners;sdurw_pathoptimization;sdurw_simulation;sdurw_opengl;sdurw_assembly;sdurw_task;sdurw_calibration;sdurw_csg;sdurw_control;sdurw_proximitystrategies;sdurw;sdurw_core;sdurw_common;sdurw_math;sdurw_plugin;sdurw_kinematics;sdurw_geometry;sdurw_graphics;sdurw_graspplanning;sdurw_invkin;sdurw_kinematics;sdurw_loaders;sdurw_pathplanning;sdurw_trajectory;sdurw_sensor;sdurw_models;sdurw_proximity")
set(RW_BUILD_WITH_LIB_DEPEND "fcl;/usr/lib/x86_64-linux-gnu/libassimp.so;/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so;dl;Boost::filesystem;Boost::serialization;Boost::system;Boost::thread;Boost::program_options;Boost::date_time;Boost::headers")

set(RW_BUILD_WITH_LIBRARY_DIRS "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWork/libs/debug;/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWork/libs/debug;/usr/lib/x86_64-linux-gnu;/usr/lib/x86_64-linux-gnu")
set(RW_BUILD_WITH_LIBRARY_SUBDIRS "x86_64-linux-gnu" "x86_64-linux-gnu/RobWork" "RobWork" "x86_64-linux-gnu/RobWork/static" "RobWork/static")
set(RW_BUILD_WITH_INCLUDE_DIR "/usr/include/eigen3;/home/zw/CLionProjects/RobWork/RobWork/src;/usr/include;/usr/include;/usr/include;/home/zw/CLionProjects/RobWork/RobWork/ext/rwyaobi/include;/home/zw/CLionProjects/RobWork/RobWork/ext/rwpqp;/usr/include/libqhull_r;/usr/include;/home/zw/CLionProjects/RobWork/RobWork/ext/csgjs/src;/usr/include;/usr/include")

