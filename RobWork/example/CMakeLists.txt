cmake_minimum_required(VERSION 3.10  )

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

project(rw-examples CXX)
# Subdirectories to process.
add_subdirectory(bfgsApp)
add_subdirectory(consoleapp)
add_subdirectory(plugin)
add_subdirectory(simplegui)
add_subdirectory(snippets)
add_subdirectory(tutorial)