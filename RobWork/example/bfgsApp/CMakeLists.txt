# ##################################################################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source files and when configuring your project.
# ##################################################################################################

# Test CMake version
cmake_minimum_required(VERSION 3.10  FATAL_ERROR )

# The name of the project. (EDIT THIS)
project(bfgsApp)

set(ROOT ${CMAKE_CURRENT_BINARY_DIR})

# optionally: point the find package in the direction of the RobWork.
set(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
find_package(RobWork REQUIRED PATHS "${RowWork_DIR}")

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
if(MSVC)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        "${ROOT}/bin"
        CACHE PATH "Runtime directory" FORCE
    )
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        "${ROOT}/libs"
        CACHE PATH "Library directory" FORCE
    )
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${ROOT}/libs"
        CACHE PATH "Archive directory" FORCE
    )
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        "${ROOT}/bin/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Runtime directory" FORCE
    )
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        "${ROOT}/libs/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Library directory" FORCE
    )
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        "${ROOT}/libs/${CMAKE_BUILD_TYPE}"
        CACHE PATH "Archive directory" FORCE
    )
endif()

# if you have additional libraries or include dirs then add them here
set(USER_LIBRARIES)
include_directories(${ROOT}/src/)
link_directories()

# And now we add any targets that we want
add_executable(bfgsTest bfgsExample.cpp)
target_link_libraries(bfgsTest ${ROBWORK_LIBRARIES})
