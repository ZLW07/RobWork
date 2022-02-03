#####################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source 
# files and when configuring your project.  
#####################################################

# Test CMake version
cmake_minimum_required(VERSION 3.10  )

# The name of the project. (EDIT THIS)
project(ConsoleApp)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
find_package(RobWork REQUIRED PATHS ${RobWork_DIR})

link_directories( ${ROBWORK_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
if(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
endif()

# And now we add any targets that we want
add_executable(SampleTest SampleTest.cpp)
target_link_libraries(SampleTest PRIVATE ${ROBWORK_LIBRARIES})
target_include_directories(SampleTest PRIVATE ${ROBWORK_INCLUDE_DIRS} )

# if you have additional libraries or include dirs then add them here
set(USER_LIBRARIES )
target_link_libraries(SampleTest PRIVATE ${USER_LIBRARIES})
target_include_directories(SampleTest PRIVATE ${USER_LIBRARIES})
link_directories( )