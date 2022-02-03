cmake_minimum_required(VERSION 3.10  )
project(PluginTest)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS} src)
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

set(FILES_CPP CACHE INTERNAL "" FORCE)
add_subdirectory(src)

add_library(PluginTest ${FILES_CPP})
message(STATUS "${ROBWORK_LIBRARIES}")
target_link_libraries(PluginTest ${ROBWORK_LIBRARIES})
