# Version of RobWork
set(RWSIM_BUILD_WITH_VERSION 22.2.3)
set(RWSIM_BUILD_WITH_VERSION_MAJOR 22)
set(RWSIM_BUILD_WITH_VERSION_MINOR 2)
set(RWSIM_BUILD_WITH_VERSION_PATCH 3)


set(RWSIM_BUILD_WITH_RW_ROOT /home/zw/CLionProjects/RobWork/RobWork/)
set(RWSIM_BUILD_WITH_RWS_ROOT /home/zw/CLionProjects/RobWork/RobWorkStudio/)
set(RWSIM_BUILD_WITH_RWSIM_ROOT /home/zw/CLionProjects/RobWork/RobWorkSim)
set(RWSIM_BUILD_WITH_BUILD_DIR "/home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim")

set(RWSIM_BUILD_WITH_LIBRARIES sdurwsim_bullet;sdurwsim_ode;sdurwsim_gui;sdurwsim)
set(RWSIM_BUILD_WITH_LIB_DEPEND /usr/lib/x86_64-linux-gnu/libBulletDynamics.so;/usr/lib/x86_64-linux-gnu/libBulletCollision.so;/usr/lib/x86_64-linux-gnu/libLinearMath.so;/usr/lib/x86_64-linux-gnu/libBulletSoftBody.so;/usr/lib/x86_64-linux-gnu/libode.so)

set(RWSIM_BUILD_WITH_INCLUDE_DIRS /home/zw/CLionProjects/RobWork/RobWorkSim/src)
set(RWSIM_BUILD_WITH_LIBRARY_SUBDIRS "x86_64-linux-gnu" "x86_64-linux-gnu/RobWork" "x86_64-linux-gnu/RobWork/static" "RobWork/static")
set(RWSIM_BUILD_WITH_LIBRARY_DIRS /home/zw/CLionProjects/RobWork/cmake-build-debug/RobWorkSim/libs/debug)

# Bullet stuff
set(RWSIM_BUILD_WITH_BULLET TRUE)
set(RWSIM_BUILD_WITH_BULLET_ROOT )
set(RWSIM_BUILD_WITH_BULLET_INCLUDE_DIR /usr/include/bullet)

# Ode stuff
set(RWSIM_BUILD_WITH_ODE TRUE)
set(RWSIM_BUILD_WITH_ODE_USE_DOUBLE )
set(RWSIM_BUILD_WITH_ODE_USE_DEBUG )
set(RWSIM_BUILD_WITH_ODE_DIR ODE_DIR-NOTFOUND)
set(RWSIM_BUILD_WITH_ODE_INCLUDE_DIR /usr/include)
