# ######################################################################################################################
# Example based on simple opengl gui that use some RobWork functionality. - the src should contain code for putting into
# libraries
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies to external projects
# ######################################################################################################################

# Test CMake version
cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

# The name of the project.
project(sdurw_viewer)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the RobWork.
set(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
find_package(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
endif()

find_package(GLUT QUIET)
if(NOT GLUT_FOUND)
    find_package(FreeGLUT REQUIRED)
    if(FreeGLUT_FOUND)
        set(GLUT_glut_LIBRARY FreeGLUT::freeglut)
        set(GLUT_FOUND ${FreeGLUT_FOUND})
    endif()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)

# add the source files
set(
    SRC_FILES
    src/SimpleGLViewer.cpp
    src/EventListener.cpp
    src/Menu.cpp
    src/MenuItem.cpp
    src/ArcBall.cpp
)

add_library(rwtools_simpleviewer ${SRC_FILES})
target_link_libraries(rwtools_simpleviewer PUBLIC RW::sdurw RW::sdurw_opengl PRIVATE ${GLUT_glut_LIBRARY})

# set the libraries we need
set(LIBRARIES rwtools_simpleviewer ${ROBWORK_LIBRARIES} ${GLUT_glut_LIBRARY})

# To create an application use
add_executable(ViewerExample ViewerExample.cpp)
target_link_libraries(ViewerExample ${LIBRARIES})

install(TARGETS ViewerExample DESTINATION bin)
