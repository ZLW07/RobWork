# if we want to use ui files add them here
set(UIS_FILES BootstrapPlugin.ui )

# to be able to include the generated ui header files
#include_directories(${CMAKE_CURRENT_BINARY_DIR}) 
#link_directories(${ROBWORKSIM_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})

if(DEFINED Qt6Core_VERSION)
    qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt_wrap_cpp(MocSrcFiles BootstrapPlugin.hpp)
    qt_add_resources(RccSrcFiles resources.qrc)
else()
    qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    qt5_wrap_cpp(MocSrcFiles BootstrapPlugin.hpp)
    qt5_add_resources(RccSrcFiles resources.qrc)
endif()

set(SrcFiles BootstrapPlugin.cpp ${UIS_OUT_H})

# The shared library to build:
add_library(BootstrapPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
target_link_libraries(BootstrapPlugin bstrap RWSIM::sdurwsim RWSIM::sdurwsim_ode RWS::sdurws RW::sdurw)
target_include_directories(BootstrapPlugin PUBLIC ${CMAKE_CURRENT_BINARY_DIR} )
